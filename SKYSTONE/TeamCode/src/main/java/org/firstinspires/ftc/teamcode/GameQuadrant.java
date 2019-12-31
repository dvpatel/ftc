package org.firstinspires.ftc.teamcode;

public enum GameQuadrant {
    LOADING_BLUE,
    BUILDING_BLUE,
    LOADING_RED,
    BUILDING_RED;

    //  help method to determine +/- directions
    final static int direction(GameQuadrant quadrant) {

        int direction = 0;

        switch (quadrant) {
            case LOADING_RED:
                direction = 1;
                break;

            case BUILDING_BLUE:
                direction = 1;
                break;

            case LOADING_BLUE:
                direction = -1;
                break;


            case BUILDING_RED:
                direction = -1;
        }

        return direction;
    }
}
