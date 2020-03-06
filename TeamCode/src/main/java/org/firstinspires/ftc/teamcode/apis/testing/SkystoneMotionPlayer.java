package org.firstinspires.ftc.teamcode.apis.testing;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.apis.testing.HardwareRepresentation.*;

import java.util.ArrayList;
import java.util.List;

/**Plays back a sequence of position targets by fetching correction data from
 * the localization system, returning coefficients that could be used to determine
 * the speed for each wheel.
 */
public class SkystoneMotionPlayer {
    Localization localization;

    //Current position in the "playlist"
    private int index = 0;

    // This only sets the multiplier for strafe speed
    public double speedMultiplier = 1;

    //The "playlist"
    private List<PositionTarget> sequence;

    public SkystoneMotionPlayer(Localization localization) {
        this.localization = localization;
        sequence = new ArrayList<>();
    }

    /**Composes corrections for rotation and translation.
     * For given correction targets.
     * @param positionTarget object containing correction targets
     * @return motionState coefficient object for mecanum drivetrain control
     */
    public MotionState getCompoundCorrection(PositionTarget positionTarget) {

        //Coefficients for mecanum drivetrain
        MotionState motionState = new MotionState();

        //Distance correction compositor
        double sumX = 0, sumY = 0;

        if(positionTarget.isTargetFrontSet){
            sumY += localization.getCorrectionSpeedFrontDistance(positionTarget.targetFront);
        }
        if(positionTarget.isTargetRearSet){
            sumY -= localization.getCorrectionSpeedRearDistance(positionTarget.targetRear);
        }
        if(positionTarget.isTargetLeftSet){
            sumX -= localization.getCorrectionSpeedLeftDistance(positionTarget.targetLeft);
        }
        if(positionTarget.isTargetRightSet){
            sumX += localization.getCorrectionSpeedRightDistance(positionTarget.targetRight);
        }

        //Vector addition
        motionState.strafeDirection = Math.atan2(sumY, sumX) - Math.PI/2;
        motionState.strafeSpeed = speedMultiplier * Range.clip(Math.sqrt(sumX * sumX + sumY * sumY), -1, 1);

        //Angle correction compositor
        motionState.rotateSpeed = 0;
        if(positionTarget.isTargetAngleLeftSet)  motionState.rotateSpeed += localization.getAngleCorrectionLeft(positionTarget.targetAngleLeft);
        if(positionTarget.isTargetAngleRightSet) motionState.rotateSpeed += localization.getAngleCorrectionRight(positionTarget.targetAngleRight);

        //Check if all corrections have been finalized
        if(motionState.rotateSpeed == 0 && sumX == 0 && sumY == 0) motionState.isDone = true;

        return motionState;
    }

    /**Composes corrections for rotation and translation.
     * For correction targets of current step in "playlist".
     * @return motionState coefficient object for mecanum drivetrain control
     */
    public MotionState getCompoundCorrection() {
        return getCompoundCorrection(sequence.get(index));
    }

    /**Checks if next step in "playlist" exists then moves to next correction step.
     */
    public void next() {
        if(index + 1 <= sequence.size() - 1) index++;
    }

    /**Gets current step in playlist
     * @return current index
     */
    public int getIndex() {
        return index;
    }

    /**Checks if step of given index in "playlist" exists then moves to that correction step.
     * @param index given index
     */
    public void setIndex(int index) {
        if(index <= sequence.size() - 1)this.index = index;
    }

    /**Adds a copy of given position target object to
     * the end of the list.
     * @param positionTarget object that contains the corrections that have to be made and the
     *                       targets for each
     */
    public void addPositionTarget(PositionTarget positionTarget){
        sequence.add(new PositionTarget(positionTarget));
    }

    /**@return positionTarget object that contains the corrections that have to be made and the
     *         targets for each
     */
    public PositionTarget getPositionTarget() {
        return sequence.get(index);
    }
}
