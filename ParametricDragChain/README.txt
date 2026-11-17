ParametricDragChain v1.4 — MATCH ORIGINAL JOINT LIMITS (-60° TO 0°)
===================================================================

CHANGED
- All script-generated revolute joints use Minimum = -60 degrees,
  Maximum = 0 degrees (assuming DegreeBendPositive is 60 degrees).
- Rerunning the script updates existing script-generated joints in-place.
- New joints created later get the same negative bend range.
- Fixed the missing-parameter fallback: if DegreeBendPositive does not
  exist, its default bend magnitude is +60 degrees (used as -60 to 0).
- Original manually made joints are not modified.

UPDATE AN EXISTING FUSION DESIGN
1. Save a COPY of your Fusion model before making changes.
2. In Fusion 360, go to Utilities > Scripts and Add-Ins and stop the script
   if it is running.
3. Replace the existing ParametricDragChain.py inside your installed
   ParametricDragChain script folder with the .py in this package. Keep
   the matching manifest or replace it with this package's manifest.
4. In the Browser, ACTIVATE the Chain_no_Clip:1 component (radio button).
5. Leave LinkCount at its current value. Run ParametricDragChain again.
   IMPORTANT: There is no need to delete all previously generated links.
6. Edit any generated PDC_Revolute joint > Motion. Confirm the checkboxes
   show Minimum = -60 deg and Maximum = 0 deg.
7. Move/drag the links into the desired pose and verify there are no collisions.

TUNING
- LinkCount controls the total number of links; rerun the script after
  changing this user parameter.
- DegreeBendPositive is used as a POSITIVE bending magnitude, normally
  60 degrees. The generated range is NEGATIVE magnitude to zero.
- FLIP_JOINT is unchanged from v1.3. Changing joint limits alone does not
  reverse the local Z-axis direction or move joints into a new pose.
  If the physical bend direction remains backward even though the Motion
  dialog reads -60 to 0 degrees, the joint coordinate orientation may need
  rebuilding with FLIP_JOINT changed. Test on a saved copy with LinkCount=6.
- The original connected links are assumed to be five (BASE_LINKS=5).

NOTE
Script is syntax-checked and the angle-setting function is smoke-tested.
It has NOT been executed inside Autodesk Fusion 360.
