package org.firstinspires.ftc.teamcode.apis.testing;

import java.lang.annotation.Target;

public class HardwareRepresentation {
    /**Keeps track of which corrections have to be done
     * and which target values do they have.
     */
    public static class PositionTarget {
        public boolean isTargetFrontSet = false;
        public double targetFront = 0;

        public boolean isTargetRearSet = false;
        public double targetRear = 0;

        public boolean isTargetLeftSet = false;
        public double targetLeft = 0;

        public boolean isTargetRightSet = false;
        public double targetRight = 0;

        public boolean isTargetAngleLeftSet = false;
        public double targetAngleLeft = 0;

        public boolean isTargetAngleRightSet = false;
        public double targetAngleRight = 0;

        public void setCorrectionTarget(CorrectionTarget correctionTarget, boolean enabled) {
            Correction correction = correctionTarget.correction;
            double target = correctionTarget.target;
            switch (correction){
                case DIST_FRONT:
                    isTargetFrontSet = enabled;
                    targetFront = target;
                    break;
                case DIST_REAR:
                    isTargetRearSet = enabled;
                    targetRear = target;
                    break;
                case DIST_LEFT:
                    isTargetLeftSet = enabled;
                    targetLeft = target;
                    break;
                case DIST_RIGHT:
                    isTargetRightSet = enabled;
                    targetRight = target;
                    break;
                case DIST_DIFF_LEFT:
                    isTargetAngleLeftSet = enabled;
                    targetAngleLeft = target;
                    break;
                case DIST_DIFF_RIGHT:
                    isTargetAngleRightSet = enabled;
                    targetAngleRight = target;
                    break;
            }
        }

        /**Enables all corrections given as parameters and sets their targets
         */
        public void setCorrectionTargets(CorrectionTarget ...correctionTargets) {
            for(CorrectionTarget correctionTarget : correctionTargets){
                setCorrectionTarget(correctionTarget, true);
            }
        }

        public PositionTarget() {
            //blank constructor
        }

        public PositionTarget(PositionTarget positionTarget){
            this.isTargetFrontSet = positionTarget.isTargetFrontSet;
            this.targetFront = positionTarget.targetFront;

            this.isTargetRearSet = positionTarget.isTargetRearSet;
            this.targetRear = positionTarget.targetRear;

            this.isTargetLeftSet = positionTarget.isTargetLeftSet;
            this.targetLeft = positionTarget.targetLeft;

            this.isTargetRightSet = positionTarget.isTargetRightSet;
            this.targetRight = positionTarget.targetRight;

            this.isTargetAngleLeftSet = positionTarget.isTargetAngleLeftSet;
            this.targetAngleLeft = positionTarget.targetAngleLeft;

            this.isTargetAngleRightSet = positionTarget.isTargetAngleRightSet;
            this.targetAngleRight = positionTarget.targetAngleRight;
        }
    }

    /**Stores coefficients for mecanum drivetrain control
     */
    public static class MotionState {
        public double strafeDirection;
        public double strafeSpeed;
        public double rotateSpeed;
        public boolean isDone = false;
    }

    /**All the correction types
     */
    public enum Correction {
        DIST_FRONT,
        DIST_REAR,
        DIST_LEFT,
        DIST_RIGHT,
        DIST_DIFF_LEFT,
        DIST_DIFF_RIGHT
    }

    /**Stores correction as type and target pair
     */
    public static class CorrectionTarget {
        public Correction correction;
        public double target;

        public CorrectionTarget(Correction correction, double target) {
            this.correction = correction;
            this.target = target;
        }
    }
}
