# Fusion 360 Python script: ParametricDragChain v1.4 (negative bending limits)
# Creates additional, independently articulated drag-chain links, up to LinkCount.
# Designed for a Chain_no_Clip:1 assembly with 5 existing connected CableChain links.
# Copyright (c) 2026. Provided as an editable example, without warranty.

import adsk.core
import adsk.fusion
import math
import traceback

# IMPORTANT: Set to the number of ALREADY CONNECTED links in your original chain.
# Your screenshot has CableChain:1 through CableChain:5 in the original chain.
BASE_LINKS = 5
DEFAULT_LINK_COUNT = 30
# DegreeBendPositive is a positive angle MAGNITUDE (typically 60 deg).
# Generated joint travel matches the original joints: -60 deg to 0 deg.
DEFAULT_BEND_MAGNITUDE_DEG = 60.0

# If the chain folds the wrong way or new links face backward, change this,
# then delete script-generated links by setting LinkCount = BASE_LINKS, run,
# change LinkCount to 30, and run again.
FLIP_JOINT = False
JOINT_OFFSET_MM = 0.0

# Keep this tag unchanged so old script-created objects and saved pivots remain recognized.
TAG = 'ParametricDragChain_v1'
ATTR_SEED_OCC = 'seed_occ_token'  # legacy: do not create new occurrence tokens
ATTR_SEED_NAME = 'seed_occ_name'   # new: safe for nested occurrences
ATTR_IN_EDGE = 'input_edge_token'
ATTR_OUT_EDGE = 'output_edge_token'
ATTR_BASE_LINKS = 'base_links'
ATTR_SEQUENCE = 'sequence'


def _attribute(obj, name):
    a = obj.attributes.itemByName(TAG, name)
    return a.value if a else None


def _save_attr(obj, name, value):
    obj.attributes.add(TAG, name, str(value))


def _native(entity):
    return entity.nativeObject or entity


def _from_token(design, token, what):
    result = design.findEntityByToken(token)
    if not result:
        raise RuntimeError(
            'Saved {} could not be found. The link geometry may have changed. '
            'Restore the original design or clear script state; see README.'.format(what)
        )
    return _native(result[0])


class SelectionCancelled(Exception):
    """The user ended the Fusion selectEntity prompt without a valid selection."""
    pass


def _select_edge(ui, prompt):
    """Prompt for a circular BRep edge, treating a canceled prompt gracefully.

    Some Fusion versions raise InternalValidationError: selections.size() > 0
    when ESC cancels selectEntity instead of returning None. Clearing selections
    first also reduces interference from previous component highlights.
    """
    try:
        ui.activeSelections.clear()
    except Exception:
        # Clearing a previous programmatic selection is unreliable in some
        # Fusion versions; don't hide a later real selection error.
        pass

    try:
        selection = ui.selectEntity(prompt, 'CircularEdges')
    except RuntimeError as exc:
        message = str(exc)
        if 'selections.size() > 0' in message or 'InternalValidationError' in message:
            raise SelectionCancelled(
                'No circular pivot edge was selected.\n\n'
                'Click OK on the instructions dialog, then LEFT-CLICK the actual '
                'circular metal edge on the chain link in the canvas.\n'
                'Do not press Esc or close the selection prompt.\n\n'
                'You can rerun the script safely; no new link was created.'
            ) from None
        raise

    if not selection:
        raise SelectionCancelled('No circular pivot edge selected; rerun and click the circular rim.')

    edge = adsk.fusion.BRepEdge.cast(selection.entity)
    if not edge:
        raise RuntimeError('Please select a circular solid edge (outline/rim), NOT a flat face or sketch.')
    occ = edge.assemblyContext
    if not occ:
        raise RuntimeError(
            'Selected edge is not in a CableChain component occurrence. '
            'Ensure Chain_no_Clip:1 is activated and pick a rim on CableChain:5.'
        )
    return _native(edge), _native(occ)


def _is_same_occurrence(a, b):
    """Compare occurrences without entityToken (which fails for nested proxies)."""
    return a.fullPathName == b.fullPathName


def _different_pivot_centers(edge_in, edge_out):
    """Check input/output pivots without comparing edge entity tokens.

    Two different concentric circles at the SAME pivot are not valid.
    Native geometry uses centimeters in the Fusion API.
    """
    a = edge_in.geometry
    b = edge_out.geometry
    if not hasattr(a, 'center') or not hasattr(b, 'center'):
        raise RuntimeError('Both selected edges must be circular pivot rims.')
    return a.center.distanceTo(b.center) > 0.001  # more than 0.01 mm


def _find_seed_occurrence(container, saved_name):
    matches = [occ for occ in container.occurrences
               if occ.name == saved_name or occ.fullPathName == saved_name]
    if len(matches) != 1:
        raise RuntimeError(
            'The original link {} could not be uniquely identified. '
            'Make sure it has not been deleted or renamed.'.format(saved_name)
        )
    return matches[0]


def _get_seed(design, ui, container):
    stored_name = _attribute(container, ATTR_SEED_NAME)
    stored_in = _attribute(container, ATTR_IN_EDGE)
    stored_out = _attribute(container, ATTR_OUT_EDGE)
    if stored_name and stored_in and stored_out:
        # Identify the original link by its local occurrence name instead of
        # requesting a token from a nested occurrence proxy.
        occ = _find_seed_occurrence(container, stored_name)
        e_in = adsk.fusion.BRepEdge.cast(_from_token(design, stored_in, 'input pivot edge'))
        e_out = adsk.fusion.BRepEdge.cast(_from_token(design, stored_out, 'output pivot edge'))
        if not (occ and e_in and e_out):
            raise RuntimeError('Stored template geometry is not a valid link and circular edges.')
        return occ, e_in, e_out

    ui.messageBox(
        'Select TWO circular pivot edges on the SAME LAST EXISTING LINK,\n'
        'usually CableChain:5.\n\n'
        '1. INPUT: the pivot already connected to CableChain:4.\n'
        '2. OUTPUT: the free pivot where CableChain:6 should attach.\n\n'
        'Select the circular rim of the pivot, not the flat face.\n'
        'Other loose copy/pasted links are NOT included in the base count.\n\n'
        'After clicking OK, Fusion will show a selection prompt.\n'
        'LEFT-CLICK the circular edge directly on the model (not the joint icon).\n'
        'Do not press ESC or select a flat face.\n\n'
        'TIP: Save a COPY of your model first.',
        'Parametric Drag Chain — Choose pivots'
    )
    e_in, occ1 = _select_edge(ui, '1/2: Select the CONNECTED/INPUT circular pivot edge on your last link')
    e_out, occ2 = _select_edge(ui, '2/2: Select the FREE/OUTPUT circular pivot edge on that SAME link')
    if not _is_same_occurrence(occ1, occ2):
        raise RuntimeError('Both pivot edges MUST be on the same CableChain link occurrence.')
    if not _different_pivot_centers(e_in, e_out):
        raise RuntimeError('Select pivot circles at TWO DIFFERENT ENDS of the same link.')
    if not occ1.component.name.lower().startswith('cablechain'):
        raise RuntimeError('Select circles on a CableChain link, not on an end plate or another part.')
    # Store seed information only after all selections have passed validation.
    # Do NOT read occ1.entityToken: Fusion rejects tokens on nested proxies.
    _save_attr(container, ATTR_SEED_NAME, occ1.name)
    # e_in / e_out were normalized to native BRep edges by _select_edge.
    _save_attr(container, ATTR_IN_EDGE, e_in.entityToken)
    _save_attr(container, ATTR_OUT_EDGE, e_out.entityToken)
    _save_attr(container, ATTR_BASE_LINKS, BASE_LINKS)
    return occ1, e_in, e_out


def _managed_occurrences(container):
    generated = {}
    for occ in container.occurrences:
        seq = _attribute(occ, ATTR_SEQUENCE)
        if seq is not None:
            n = int(seq)
            if n in generated:
                raise RuntimeError('Duplicate generated sequence number {}. Stop and inspect the assembly.'.format(n))
            generated[n] = occ
    return generated


def _managed_joints(container):
    result = {}
    for joint in container.joints:
        seq = _attribute(joint, ATTR_SEQUENCE)
        if seq is not None:
            result[int(seq)] = joint
    return result


def _setup_parameter(design):
    p = design.userParameters.itemByName('LinkCount')
    if not p:
        p = design.userParameters.add(
            'LinkCount',
            adsk.core.ValueInput.createByString(str(DEFAULT_LINK_COUNT)),
            '',
            'Total articulated chain links; rerun ParametricDragChain after editing.'
        )
        if not p:
            raise RuntimeError('Unable to create LinkCount user parameter.')
    value = p.value
    total = int(round(value))
    if abs(value - total) > 1e-8 or total < BASE_LINKS or total > 200:
        raise RuntimeError('LinkCount must be a whole number between {} and 200.'.format(BASE_LINKS))
    bend = design.userParameters.itemByName('DegreeBendPositive')
    bend_magnitude = float(bend.value) if bend else math.radians(DEFAULT_BEND_MAGNITUDE_DEG)
    if bend_magnitude <= 0 or bend_magnitude >= 2 * math.pi:
        raise RuntimeError('DegreeBendPositive should be between 0 and 360 degrees (exclusive).')
    return total, bend_magnitude


def _set_revolute_limits(joint, bend_magnitude_radians):
    """Match existing CableChain joints: minimum -bend magnitude, maximum 0."""
    motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
    if not motion:
        raise RuntimeError('Joint {} is not revolute.'.format(joint.name))
    limits = motion.rotationLimits
    limits.isMinimumValueEnabled = True
    limits.minimumValue = -bend_magnitude_radians
    limits.isMaximumValueEnabled = True
    limits.maximumValue = 0.0


def _make_revolute(container, previous_out_edge, new_input_edge, seq, bend_magnitude_radians):
    pivot1 = adsk.fusion.JointGeometry.createByCurve(
        previous_out_edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint
    )
    pivot2 = adsk.fusion.JointGeometry.createByCurve(
        new_input_edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint
    )
    if not (pivot1 and pivot2):
        raise RuntimeError('Unable to make joint centers. Select fully circular pivot edges.')
    joints = container.joints
    joint_input = joints.createInput(pivot1, pivot2)
    if not joint_input:
        raise RuntimeError('Could not create JointInput. Are the link occurrences in the active component?')
    joint_input.isFlipped = FLIP_JOINT
    joint_input.angle = adsk.core.ValueInput.createByString('0 deg')
    if JOINT_OFFSET_MM:
        joint_input.offset = adsk.core.ValueInput.createByString('{} mm'.format(JOINT_OFFSET_MM))
    joint_input.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    joint = joints.add(joint_input)
    if not joint:
        raise RuntimeError('Fusion rejected joint at link {}.'.format(seq))
    joint.name = 'PDC_Revolute_{:02d}'.format(seq)
    _save_attr(joint, ATTR_SEQUENCE, seq)
    _set_revolute_limits(joint, bend_magnitude_radians)
    return joint


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a Fusion 360 Design before running this script.')
            return

        container = design.activeComponent
        if not container or 'chain_no_clip' not in container.name.lower():
            ui.messageBox(
                'First ACTIVATE the Chain_no_Clip:1 component using the radio button\n'
                'beside its name in the Browser. Then run the script again.\n\n'
                'This prevents new links from being inserted at the top level.',
                'Activate the chain component'
            )
            return
        old_base = _attribute(container, ATTR_BASE_LINKS)
        if old_base and int(old_base) != BASE_LINKS:
            raise RuntimeError('Saved base count was {} but script now uses {}. Restore that setting.'.format(
                old_base, BASE_LINKS))

        target, bend_magnitude = _setup_parameter(design)
        created = _managed_occurrences(container)
        expected = set(range(BASE_LINKS + 1, BASE_LINKS + len(created) + 1))
        if set(created) != expected:
            raise RuntimeError('The script-generated links have nonconsecutive sequence numbers. Inspect the assembly.')

        # Removing generated links does NOT remove any of the original user's links.
        if target < BASE_LINKS + len(created):
            existing_joints = _managed_joints(container)
            for seq in sorted(created.keys(), reverse=True):
                if seq <= target:
                    continue
                joint = existing_joints.get(seq)
                if joint and joint.isValid:
                    joint.deleteMe()
                created[seq].deleteMe()
            created = _managed_occurrences(container)

        for joint in _managed_joints(container).values():
            _set_revolute_limits(joint, bend_magnitude)

        if target == BASE_LINKS + len(created):
            ui.messageBox(
                'Already at {} links ({} original + {} script-generated).\n'
                'All generated revolute joint limits are now {:.1f} to 0.0 degrees.'.format(
                    target, BASE_LINKS, len(created), -math.degrees(bend_magnitude)
                ),
                'Drag chain up to date'
            )
            return

        seed_occ, native_input, native_output = _get_seed(design, ui, container)
        if not seed_occ.isValid or seed_occ.component is None:
            raise RuntimeError('Original template CableChain component is invalid.')

        if created:
            last_seq = max(created)
            prev_occ = created[last_seq]
        else:
            prev_occ = seed_occ

        for seq in range(BASE_LINKS + len(created) + 1, target + 1):
            # Paste (linked): all added occurrences share the parametric link definition.
            new_occ = container.occurrences.addExistingComponent(
                seed_occ.component, seed_occ.transform2.copy()
            )
            if not new_occ:
                raise RuntimeError('Could not add link {}.'.format(seq))
            try:
                prev_out_proxy = native_output.createForAssemblyContext(prev_occ)
                next_in_proxy = native_input.createForAssemblyContext(new_occ)
                if not prev_out_proxy or not next_in_proxy:
                    raise RuntimeError('Could not create the geometry in the assembly context.')
                _make_revolute(container, prev_out_proxy, next_in_proxy, seq, bend_magnitude)
                # Occurrence.name is READ ONLY in the Fusion API.
                # Fusion assigns the displayed name (e.g. CableChain:10).
                # Track script-managed copies using a per-occurrence attribute instead.
                _save_attr(new_occ, ATTR_SEQUENCE, seq)
                prev_occ = new_occ
                if seq % 5 == 0:
                    adsk.doEvents()
            except Exception:
                # Clean up a link that was just created but never successfully completed.
                for j in _managed_joints(container).values():
                    if _attribute(j, ATTR_SEQUENCE) == str(seq):
                        j.deleteMe()
                if new_occ.isValid:
                    new_occ.deleteMe()
                raise

        ui.messageBox(
            'Finished. Total = {} links\n'
            'Original links = {}\n'
            'Script-created links = {}\n'
            'Revolute Z-axis joint limits = {:.1f} to 0 deg\n\n'
            'Edit LinkCount in Change Parameters and rerun to add/remove links.\n'
            'If the first new link overlaps or flips, undo, change FLIP_JOINT, '
            'and test again with LinkCount = 6 first.'.format(
                target, BASE_LINKS, target - BASE_LINKS, -math.degrees(bend_magnitude)
            ),
            'Parametric Drag Chain Complete'
        )
    except SelectionCancelled as exc:
        if ui:
            ui.messageBox(str(exc), 'Drag Chain — Selection Not Completed')
    except Exception:
        if ui:
            ui.messageBox('ParametricDragChain error:\n\n' + traceback.format_exc())
