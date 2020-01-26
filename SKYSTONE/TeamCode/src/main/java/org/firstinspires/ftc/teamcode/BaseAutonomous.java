package org.firstinspires.ftc.teamcode;

import org.blueprint.ftc.core.AbstractLinearOpMode;

public abstract class BaseAutonomous extends AbstractLinearOpMode {

    private boolean quadrantSelected;

    protected boolean isQuadrantSelected() {
        return this.quadrantSelected;
    }

    protected void setQuadrantSelected() {
        this.quadrantSelected = true;
    }

    //  method helps to identify location of robot
    protected GameQuadrant selectGameQuadrant() {

        // Wait until gamepad a or b is press for zone selection;
        GameQuadrant gq = null;
        boolean isLoadingZone = false;
        boolean isBuildingZone = false;

        while (!isQuadrantSelected()) {

            if (!isLoadingZone && !isBuildingZone) {

                telemetry.addData("Instructions", "Press A for LOADING ZONE; Press Y for BUILDING ZONE");

                if (gamepad1.a) {
                    isLoadingZone = true;
                } else if (gamepad1.y) {
                    isBuildingZone = true;
                }
            }

            if (isLoadingZone || isBuildingZone && gq == null) {

                telemetry.addData("Instructions", "Press X for BLUE ALLIANCE; Press B for RED ALLIANCE");

                if (gamepad1.b) {
                    //  red alliance

                    if (isLoadingZone) {
                        gq = GameQuadrant.LOADING_RED;
                    } else if (isBuildingZone) {
                        gq = GameQuadrant.BUILDING_RED;
                    }

                } else if (gamepad1.x) {
                    //  blue alliance;

                    if (isLoadingZone) {
                        gq = GameQuadrant.LOADING_BLUE;
                    } else if (isBuildingZone) {
                        gq = GameQuadrant.BUILDING_BLUE;
                    }
                }
            }

            telemetry.update();

            if (gq != null) {
                this.setQuadrantSelected();
            }
        }

        return gq;
    }


}
