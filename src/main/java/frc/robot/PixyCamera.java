// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

package frc.robot;


public class PixyCamera {

    private static int X1 = 156;    // Distance 1
    private static int Y1 = 49;     // Y_value off Pixy

    private static int X2 = 200;    // Distance 2
    private static int Y2 = 65;     // Y_value off Pixy

    private static int m = (X2-X1)/(Y2-Y1);
    private static int b = X2-(m*Y2);

    public static int getDistancetoTarget(int Y_Value) {
        return ((m * Y_Value) + b);
    }

}
