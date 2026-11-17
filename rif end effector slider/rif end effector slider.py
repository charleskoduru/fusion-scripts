import adsk.core
import adsk.fusion
import traceback
import math

CMD_ID = 'RIF_MultiJoint_Live_Controller_v3'
CMD_NAME = 'RIF Live Joint Controller'
CMD_DESCRIPTION = 'Live multi-joint controller with limits and joint references.'

DEFAULT_MIN_DEG = -90.0
DEFAULT_MAX_DEG = 90.0

_app = None
_ui = None
_design = None
_cmd_def = None
_handlers = []
_rows = []
_label_to_index = {}
_syncing = False


# ============================================================
# BASIC HELPERS
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def rad2deg(v):
    return math.degrees(v)


def deg2rad(v):
    return math.radians(v)


def get_motion(joint):
    return adsk.fusion.RevoluteJointMotion.cast(
        joint.jointMotion
    )


# ============================================================
# FIND REVOLUTE JOINTS
# ============================================================

def find_revolute_joints(design):

    result = []
    seen = set()

    comps = design.allComponents

    for ci in range(comps.count):

        comp = comps.item(ci)

        for ji in range(comp.joints.count):

            joint = comp.joints.item(ji)

            motion = get_motion(joint)

            if not motion:
                continue

            try:
                key = joint.entityToken
            except:
                key = f'{ci}:{ji}:{joint.name}'

            if key in seen:
                continue

            seen.add(key)
            result.append(joint)

    return result


# ============================================================
# REFERENCE-JOINT HELPERS
# ============================================================

def source_index(row):

    item = row['source'].selectedItem

    if not item:
        return None

    if item.name == '<None>':
        return None

    return _label_to_index.get(
        item.name
    )


# ============================================================
# SLIDER LIMIT GUI
# ============================================================

def set_slider_range(row):

    global _syncing

    mn = row['min'].value
    mx = row['max'].value

    slider = row['slider']

    if mn >= mx:

        slider.isEnabled = False

        row['status'].formattedText = (
            '<b>Invalid limits:</b> '
            'Min must be less than Max.'
        )

        return False

    old_min = slider.minimumValue
    old_max = slider.maximumValue

    target = clamp(
        slider.valueOne,
        mn,
        mx
    )

    try:

        _syncing = True

        # Expand first.
        if mn < old_min:
            slider.minimumValue = mn

        if mx > old_max:
            slider.maximumValue = mx

        slider.valueOne = target

        # Then shrink.
        if mn > old_min:
            slider.minimumValue = mn

        if mx < old_max:
            slider.maximumValue = mx

        slider.setText(
            f'{rad2deg(mn):.0f}°',
            f'{rad2deg(mx):.0f}°'
        )

        return True

    finally:

        _syncing = False


# ============================================================
# ENABLE / DISABLE GUI ELEMENTS
# ============================================================

def update_control_state(row):

    active = row['active'].value
    linked = row['link'].value

    valid_limits = (
        row['min'].value <
        row['max'].value
    )

    # Own slider is active only if this
    # joint is NOT following another joint.
    row['slider'].isEnabled = (
        active
        and not linked
        and valid_limits
    )

    # Reference controls only make sense
    # when Follow Reference is enabled.
    row['source'].isEnabled = (
        active and linked
    )

    row['ratio'].isEnabled = (
        active and linked
    )

    row['offset'].isEnabled = (
        active and linked
    )

    if not active:

        row['status'].formattedText = (
            'Disabled — this joint will '
            'not be changed.'
        )

    elif linked:

        src = row['source'].selectedItem

        src_name = (
            src.name
            if src
            else '<None>'
        )

        row['status'].formattedText = (
            f'Linked to <b>{src_name}</b>.'
        )

    else:

        row['status'].formattedText = (
            'Live slider control enabled.'
        )


# ============================================================
# CONFIGURATION VALIDATION
# ============================================================

def validate_configuration(
    return_message=False
):

    # --------------------------------------------------------
    # Check limits and missing references
    # --------------------------------------------------------

    for i, row in enumerate(_rows):

        if not row['active'].value:
            continue

        if (
            row['min'].value >=
            row['max'].value
        ):

            msg = (
                f'{row["joint"].name}: '
                'minimum limit must be less '
                'than maximum limit.'
            )

            if return_message:
                return False, msg

            return False

        if row['link'].value:

            src = source_index(row)

            if src is None:

                msg = (
                    f'{row["joint"].name}: '
                    'choose a reference joint.'
                )

                if return_message:
                    return False, msg

                return False

            if src == i:

                msg = (
                    f'{row["joint"].name}: '
                    'a joint cannot reference itself.'
                )

                if return_message:
                    return False, msg

                return False

    # --------------------------------------------------------
    # Detect circular references
    #
    # Example:
    #
    # Joint 2 follows Joint 3
    # Joint 3 follows Joint 2
    #
    # That would create an infinite loop.
    # --------------------------------------------------------

    def visit(
        i,
        visiting,
        visited
    ):

        if i in visited:
            return None

        if i in visiting:

            return (
                'Reference cycle detected. '
                'A linked joint chain cannot '
                'loop back on itself.'
            )

        row = _rows[i]

        if (
            not row['active'].value
            or
            not row['link'].value
        ):

            visited.add(i)

            return None

        src = source_index(row)

        if src is None:

            visited.add(i)

            return None

        visiting.add(i)

        err = visit(
            src,
            visiting,
            visited
        )

        visiting.remove(i)
        visited.add(i)

        return err

    visited = set()

    for i in range(len(_rows)):

        err = visit(
            i,
            set(),
            visited
        )

        if err:

            if return_message:
                return False, err

            return False

    if return_message:
        return True, ''

    return True


# ============================================================
# CALCULATE TARGET ANGLE
# ============================================================

def desired_angle(
    i,
    memo,
    stack
):

    if i in memo:
        return memo[i]

    if i in stack:

        raise RuntimeError(
            'Reference cycle detected.'
        )

    row = _rows[i]

    motion = get_motion(
        row['joint']
    )

    # --------------------------------------------------------
    # Disabled joints can still be used as references.
    # Their current CAD position will be used.
    # --------------------------------------------------------

    if not row['active'].value:

        value = motion.rotationValue

        memo[i] = value

        return value

    mn = row['min'].value
    mx = row['max'].value

    # --------------------------------------------------------
    # Normal independent slider joint
    # --------------------------------------------------------

    if not row['link'].value:

        value = clamp(
            row['slider'].valueOne,
            mn,
            mx
        )

        memo[i] = value

        return value

    # --------------------------------------------------------
    # Linked joint
    # --------------------------------------------------------

    src = source_index(row)

    if src is None:

        raise RuntimeError(
            f'{row["joint"].name}: '
            'no reference joint selected.'
        )

    stack.add(i)

    reference = desired_angle(
        src,
        memo,
        stack
    )

    stack.remove(i)

    # ========================================================
    # JOINT REFERENCE EQUATION
    #
    # target =
    # reference angle * ratio + offset
    #
    # Examples:
    #
    # Equal:
    # ratio = 1
    # offset = 0
    #
    # Opposite:
    # ratio = -1
    # offset = 0
    #
    # Half speed:
    # ratio = 0.5
    #
    # Offset:
    # ratio = 1
    # offset = 20 degrees
    # ========================================================

    value = (
        reference *
        row['ratio'].value
        +
        row['offset'].value
    )

    value = clamp(
        value,
        mn,
        mx
    )

    memo[i] = value

    return value


# ============================================================
# APPLY MODEL CHANGES
#
# THIS IS CALLED FROM EXECUTE PREVIEW.
# ============================================================

def apply_from_gui():

    ok, message = (
        validate_configuration(True)
    )

    if not ok:

        for row in _rows:

            if row['active'].value:

                row['status'].formattedText = (
                    f'<b>Cannot preview:</b> '
                    f'{message}'
                )

        return False

    # --------------------------------------------------------
    # Temporarily disable limits before moving.
    #
    # This prevents a newly tightened range from blocking
    # the joint before we move it inside the new range.
    # --------------------------------------------------------

    for row in _rows:

        if not row['active'].value:
            continue

        motion = get_motion(
            row['joint']
        )

        limits = (
            motion.rotationLimits
        )

        limits.isMinimumValueEnabled = False
        limits.isMaximumValueEnabled = False

        limits.minimumValue = (
            row['min'].value
        )

        limits.maximumValue = (
            row['max'].value
        )

    # --------------------------------------------------------
    # Determine every desired joint angle
    # --------------------------------------------------------

    memo = {}

    for i, row in enumerate(_rows):

        if not row['active'].value:
            continue

        target = desired_angle(
            i,
            memo,
            set()
        )

        motion = get_motion(
            row['joint']
        )

        # This is equivalent to Drive Joint.
        motion.rotationValue = target

    # --------------------------------------------------------
    # Re-enable limits after positions are valid.
    # --------------------------------------------------------

    for row in _rows:

        if not row['active'].value:
            continue

        limits = (
            get_motion(
                row['joint']
            ).rotationLimits
        )

        limits.isMinimumValueEnabled = True
        limits.isMaximumValueEnabled = True

    # Force Fusion to solve the assembly.
    _design.computeAll()

    # Refresh canvas immediately.
    _app.activeViewport.refresh()

    # --------------------------------------------------------
    # Update GUI status
    # --------------------------------------------------------

    for i, row in enumerate(_rows):

        motion = get_motion(
            row['joint']
        )

        actual = rad2deg(
            motion.rotationValue
        )

        if not row['active'].value:

            row['status'].formattedText = (
                f'Disabled | Current: '
                f'<b>{actual:.1f}°</b>'
            )

            continue

        target = rad2deg(
            memo.get(
                i,
                motion.rotationValue
            )
        )

        if row['link'].value:

            src = source_index(row)

            src_name = (
                _rows[src]['joint'].name
                if src is not None
                else '<None>'
            )

            ratio = (
                row['ratio'].value
            )

            offset = rad2deg(
                row['offset'].value
            )

            row['status'].formattedText = (
                f'Following '
                f'<b>{src_name}</b><br>'
                f'Target: '
                f'<b>{target:.1f}°</b> | '
                f'Actual: '
                f'<b>{actual:.1f}°</b><br>'
                f'Rule: reference × '
                f'{ratio:.3g} '
                f'+ {offset:.1f}°'
            )

        else:

            row['status'].formattedText = (
                f'Live target: '
                f'<b>{target:.1f}°</b> | '
                f'Actual: '
                f'<b>{actual:.1f}°</b>'
            )

    return True


# ============================================================
# INPUT CHANGED
#
# IMPORTANT:
# ONLY MODIFY GUI HERE.
#
# DO NOT MODIFY THE FUSION MODEL HERE.
# ============================================================

class InputChangedHandler(
    adsk.core.InputChangedEventHandler
):

    def __init__(self):
        super().__init__()

    def notify(self, args):

        global _syncing

        try:

            if _syncing:
                return

            changed = args.input

            if not changed:
                return

            for row in _rows:

                # --------------------------------------------
                # Limits edited
                # --------------------------------------------

                if changed.id in (
                    row['min'].id,
                    row['max'].id
                ):

                    set_slider_range(row)

                    update_control_state(row)

                    return

                # --------------------------------------------
                # Checkbox or reference changed
                # --------------------------------------------

                if changed.id in (
                    row['active'].id,
                    row['link'].id,
                    row['source'].id
                ):

                    update_control_state(row)

                    return

                # --------------------------------------------
                # Slider / ratio / offset
                #
                # We don't modify geometry here.
                # Fusion automatically fires executePreview.
                # --------------------------------------------

                if changed.id in (
                    row['slider'].id,
                    row['ratio'].id,
                    row['offset'].id
                ):

                    return

        except:

            if _ui:

                _ui.messageBox(
                    'Input change failed:\n\n'
                    +
                    traceback.format_exc()
                )


# ============================================================
# LIVE PREVIEW
# ============================================================

class ExecutePreviewHandler(
    adsk.core.CommandEventHandler
):

    def __init__(self):
        super().__init__()

    def notify(self, args):

        try:

            # THIS is what makes the robot move
            # while you're dragging the slider.
            apply_from_gui()

        except:

            if _ui:

                _ui.messageBox(
                    'Preview failed:\n\n'
                    +
                    traceback.format_exc()
                )


# ============================================================
# KEEP POSITIONS
# ============================================================

class ExecuteHandler(
    adsk.core.CommandEventHandler
):

    def __init__(self):
        super().__init__()

    def notify(self, args):

        try:

            if not apply_from_gui():

                args.executeFailed = True

                args.executeFailedMessage = (
                    'Fix the invalid limits/'
                    'reference configuration.'
                )

        except:

            args.executeFailed = True

            args.executeFailedMessage = (
                traceback.format_exc()
            )


# ============================================================
# VALIDATE
# ============================================================

class ValidateHandler(
    adsk.core.ValidateInputsEventHandler
):

    def __init__(self):
        super().__init__()

    def notify(self, args):

        try:

            args.areInputsValid = (
                validate_configuration(False)
            )

        except:

            args.areInputsValid = False


# ============================================================
# CLOSE COMMAND
# ============================================================

class DestroyHandler(
    adsk.core.CommandEventHandler
):

    def __init__(self):
        super().__init__()

    def notify(self, args):

        try:

            existing = (
                _ui.commandDefinitions.itemById(
                    CMD_ID
                )
            )

            if existing:

                existing.deleteMe()

        except:

            pass

        adsk.terminate()


# ============================================================
# CREATE GUI
# ============================================================

class CommandCreatedHandler(
    adsk.core.CommandCreatedEventHandler
):

    def __init__(self):
        super().__init__()

    def notify(self, args):

        global _rows
        global _label_to_index

        try:

            cmd = args.command

            cmd.okButtonText = (
                'Keep Positions'
            )

            cmd.cancelButtonText = (
                'Revert'
            )

            cmd.setDialogInitialSize(
                560,
                720
            )

            inputs = (
                cmd.commandInputs
            )

            joints = (
                find_revolute_joints(
                    _design
                )
            )

            if not joints:

                _ui.messageBox(
                    'No revolute joints '
                    'were found in this design.'
                )

                return

            # =================================================
            # HEADER
            # =================================================

            header = (
                inputs.addTextBoxCommandInput(
                    'header',
                    '',
                    (
                        '<b>RIF LIVE JOINT '
                        'CONTROLLER</b><br>'
                        'Check Control Joint to move it. '
                        'Use Follow Reference to '
                        'couple joints.'
                    ),
                    3,
                    True
                )
            )

            header.isFullWidth = True

            help_box = (
                inputs.addTextBoxCommandInput(
                    'linkHelp',
                    '',
                    (
                        '<b>Reference equation:</b> '
                        'This joint = Reference × '
                        'Ratio + Offset<br>'
                        'Equal motion: Ratio = 1, '
                        'Offset = 0° &nbsp;&nbsp;|'
                        '&nbsp;&nbsp; '
                        'Opposite motion: '
                        'Ratio = -1, Offset = 0°'
                    ),
                    3,
                    True
                )
            )

            help_box.isFullWidth = True

            _rows = []
            _label_to_index = {}

            # =================================================
            # CREATE JOINT LABELS
            # =================================================

            labels = []

            for i, joint in enumerate(
                joints
            ):

                label = (
                    f'{i + 1}: '
                    f'{joint.name}'
                )

                labels.append(label)

                _label_to_index[
                    label
                ] = i

            # =================================================
            # BUILD ONE CONTROL GROUP PER JOINT
            # =================================================

            for i, joint in enumerate(
                joints
            ):

                motion = get_motion(
                    joint
                )

                current = (
                    motion.rotationValue
                )

                existing_limits = (
                    motion.rotationLimits
                )

                # ---------------------------------------------
                # Initial minimum
                # ---------------------------------------------

                if (
                    existing_limits.
                    isMinimumValueEnabled
                ):

                    mn = (
                        existing_limits.
                        minimumValue
                    )

                else:

                    mn = deg2rad(
                        DEFAULT_MIN_DEG
                    )

                # ---------------------------------------------
                # Initial maximum
                # ---------------------------------------------

                if (
                    existing_limits.
                    isMaximumValueEnabled
                ):

                    mx = (
                        existing_limits.
                        maximumValue
                    )

                else:

                    mx = deg2rad(
                        DEFAULT_MAX_DEG
                    )

                # Make sure current position
                # is visible on the slider.

                if current < mn:
                    mn = current

                if current > mx:
                    mx = current

                if mn >= mx:

                    mn = (
                        current -
                        deg2rad(90)
                    )

                    mx = (
                        current +
                        deg2rad(90)
                    )

                # =============================================
                # JOINT GROUP
                # =============================================

                group = (
                    inputs.addGroupCommandInput(
                        f'group_{i}',
                        labels[i]
                    )
                )

                group.isExpanded = (
                    i == 0
                )

                c = group.children

                # =============================================
                # ACTIVE CHECKBOX
                # =============================================

                active = (
                    c.addBoolValueInput(
                        f'active_{i}',
                        'Control Joint',
                        True,
                        '',
                        False
                    )
                )

                # =============================================
                # MIN / MAX LIMIT
                # =============================================

                min_in = (
                    c.addValueInput(
                        f'min_{i}',
                        'Minimum Limit',
                        'deg',
                        adsk.core.ValueInput.
                        createByReal(mn)
                    )
                )

                max_in = (
                    c.addValueInput(
                        f'max_{i}',
                        'Maximum Limit',
                        'deg',
                        adsk.core.ValueInput.
                        createByReal(mx)
                    )
                )

                # =============================================
                # LIVE SLIDER
                # =============================================

                slider = (
                    c.addFloatSliderCommandInput(
                        f'slider_{i}',
                        'Joint Angle',
                        'deg',
                        mn,
                        mx,
                        False
                    )
                )

                slider.valueOne = clamp(
                    current,
                    mn,
                    mx
                )

                slider.setText(
                    f'{rad2deg(mn):.0f}°',
                    f'{rad2deg(mx):.0f}°'
                )

                slider.isEnabled = False

                # =============================================
                # FOLLOW JOINT
                # =============================================

                link = (
                    c.addBoolValueInput(
                        f'link_{i}',
                        'Follow Reference Joint',
                        True,
                        '',
                        False
                    )
                )

                # =============================================
                # REFERENCE JOINT DROPDOWN
                # =============================================

                source = (
                    c.addDropDownCommandInput(
                        f'source_{i}',
                        'Reference Joint',
                        adsk.core.DropDownStyles.
                        TextListDropDownStyle
                    )
                )

                source.listItems.add(
                    '<None>',
                    True,
                    ''
                )

                for j, label in enumerate(
                    labels
                ):

                    if j != i:

                        source.listItems.add(
                            label,
                            False,
                            ''
                        )

                source.isEnabled = False

                # =============================================
                # RATIO
                # =============================================

                ratio = (
                    c.addValueInput(
                        f'ratio_{i}',
                        'Ratio',
                        '',
                        adsk.core.ValueInput.
                        createByReal(1.0)
                    )
                )

                ratio.isEnabled = False

                # =============================================
                # ANGLE OFFSET
                # =============================================

                offset = (
                    c.addValueInput(
                        f'offset_{i}',
                        'Offset',
                        'deg',
                        adsk.core.ValueInput.
                        createByString(
                            '0 deg'
                        )
                    )
                )

                offset.isEnabled = False

                # =============================================
                # STATUS
                # =============================================

                status = (
                    c.addTextBoxCommandInput(
                        f'status_{i}',
                        'Status',
                        (
                            'Disabled | Current: '
                            f'<b>'
                            f'{rad2deg(current):.1f}°'
                            f'</b>'
                        ),
                        3,
                        True
                    )
                )

                row = {

                    'index': i,

                    'joint': joint,

                    'active': active,

                    'min': min_in,

                    'max': max_in,

                    'slider': slider,

                    'link': link,

                    'source': source,

                    'ratio': ratio,

                    'offset': offset,

                    'status': status,
                }

                _rows.append(row)

            # =================================================
            # FOOTER
            # =================================================

            footer = (
                inputs.addTextBoxCommandInput(
                    'footer',
                    '',
                    (
                        '<b>Keep Positions</b> '
                        'commits the final positions '
                        'and limits. '
                        '<b>Revert</b> cancels '
                        'the preview and returns '
                        'to the original model.'
                    ),
                    2,
                    True
                )
            )

            footer.isFullWidth = True

            # =================================================
            # EVENTS
            # =================================================

            on_input = (
                InputChangedHandler()
            )

            cmd.inputChanged.add(
                on_input
            )

            _handlers.append(
                on_input
            )

            # ---------------------------------------------
            # THIS EVENT MAKES THE MOTION LIVE
            # ---------------------------------------------

            on_preview = (
                ExecutePreviewHandler()
            )

            cmd.executePreview.add(
                on_preview
            )

            _handlers.append(
                on_preview
            )

            on_execute = (
                ExecuteHandler()
            )

            cmd.execute.add(
                on_execute
            )

            _handlers.append(
                on_execute
            )

            on_validate = (
                ValidateHandler()
            )

            cmd.validateInputs.add(
                on_validate
            )

            _handlers.append(
                on_validate
            )

            on_destroy = (
                DestroyHandler()
            )

            cmd.destroy.add(
                on_destroy
            )

            _handlers.append(
                on_destroy
            )

        except:

            if _ui:

                _ui.messageBox(
                    'Command creation failed:\n\n'
                    +
                    traceback.format_exc()
                )


# ============================================================
# RUN
# ============================================================

def run(context):

    global _app
    global _ui
    global _design
    global _cmd_def

    try:

        _app = (
            adsk.core.Application.get()
        )

        _ui = (
            _app.userInterface
        )

        _design = (
            adsk.fusion.Design.cast(
                _app.activeProduct
            )
        )

        if not _design:

            _ui.messageBox(
                'Open a Fusion design first.'
            )

            return

        existing = (
            _ui.commandDefinitions.itemById(
                CMD_ID
            )
        )

        if existing:
            existing.deleteMe()

        _cmd_def = (
            _ui.commandDefinitions.
            addButtonDefinition(
                CMD_ID,
                CMD_NAME,
                CMD_DESCRIPTION
            )
        )

        created = (
            CommandCreatedHandler()
        )

        _cmd_def.commandCreated.add(
            created
        )

        _handlers.append(
            created
        )

        _cmd_def.execute()

        adsk.autoTerminate(False)

    except:

        if _ui:

            _ui.messageBox(
                'RIF controller failed:\n\n'
                +
                traceback.format_exc()
            )


# ============================================================
# STOP
# ============================================================

def stop(context):

    try:

        if _ui:

            existing = (
                _ui.commandDefinitions.
                itemById(
                    CMD_ID
                )
            )

            if existing:

                existing.deleteMe()

    except:

        pass