package frc.robot.Util ; 


public class AimBotCode {

    private static double getAimBotAngle(double Dx , double Dy , double currentAngle1) {
    final double l1 = 9.055662; // in
    final double l2 = 7.0; // in
    final  double l3 = 8.0; // in
    final double l4 = 11.0; // in

    double inputAngle = 0.0;

    // HEIGHT
    double h = 1.0025; // height difference between linkages

    // Distance
  final  double robot_distance_x = Dx; // m
  final  double robot_distance_y = Dy; // m

    // More Setup
 final   double angleInitComp = 6.0;
    double currentInputAngle = currentAngle1; // the current position on the input linkage
    double target_hitangle = 0.0;

    // Declarations and Constants
    double psi = 90 - Math.toDegrees(Math.acos((h / l1)) ); // angle sign accounting for non-parallel linkages
   final  double g = 9.81;
  final  double angle_speaker = 90 - Math.toDegrees(Math.atan(robot_distance_x / robot_distance_y));

    // New calculation for centroid of robot to the top of the speaker angle

    double shiftDist = 14.0  * 0.0254;     // both sides, but unilateral total for one side.
    double aimPointYShift = 0.0;

  if (angle_speaker < 60.0) {
       aimPointYShift = Math.sin(Math.toRadians(angle_speaker + 10)) * shiftDist  ; // the 10 is kind of arbitrary
       }
  else{ 
       aimPointYShift = shiftDist  + shiftDist ;
 }
    
    double r = Math.sqrt(Math.pow(robot_distance_x, 2) + Math.pow((robot_distance_y), 2));

    double target_distance = r;
    double current_distance = target_distance; // guess

    double target_height = 2.105;
    double hTar = 1.0;
    double place = Math.signum(currentInputAngle);

    double compFac = current_distance * 0.001; // somewhat dependent on angle.

    double aimPoint = target_height + compFac;
    double angle_centroid_speaker = Math.atan(aimPoint / current_distance) + Math.toRadians(angleInitComp); // a good approximation for the final shooter angle

    double shooterGuess = Math.toDegrees(angle_centroid_speaker);


    var shooterAngleApprox = shooterGuess;
    double shooter_dist = current_distance;

    double point = aimPoint;

  double a5, l7y, c4x, c4y, y11, x11, y0, x0, calcAngle, v0, absDiff, absInputAngle, a4 , xL4, yL4 , y01 , x111, Ox , y111 , ang0 , phi, gamma,beta , a7 , a8 , l7x;  // huge declaration chain
  double[] angles;
          double c4 = 3.25;

       absDiff = Math.abs(hTar - aimPoint);
       absInputAngle = Math.abs(inputAngle);

    while (absDiff >= 0.0001 && absInputAngle < 720.0) {
        inputAngle = getInputAngle(shooterAngleApprox, place);
         angles = getShooterAngle(inputAngle);



         ang0 = angles[0];
         phi = angles[1];
         gamma = angles[2];
         beta = angles[3];



        if (inputAngle > 0) {
            a4 = 360 - beta - (180 - inputAngle) - gamma + psi;
            xL4 = -1.0 * Math.cos(Math.toRadians(a4)) * l4;
            yL4 = Math.sin(Math.toRadians(a4)) * l4;
            y01 = 6.22;
            Ox = -1.25;


        } 
        else {
            y01 = 7.22;
            Ox = -10.25;
            x111 = l1 * Math.cos(Math.toRadians(psi));
            y111 = l1 * -1 * Math.sin(Math.toRadians(psi));

            a4 = 180 - phi - gamma;
             a7 = Math.toDegrees(Math.asin(h / l1));
             a8 = a4 - psi + a7;

             l7x = -1 * Math.cos(Math.toRadians(a8)) * l4;
             l7y = Math.sin(Math.toRadians(a8)) * l4;

             a5 = 180 + inputAngle;

            xL4 = l7x + x111;
            yL4 = l7y + y111;
        }

        c4x = c4 * Math.cos(Math.toRadians(ang0));
        c4y = c4 * Math.sin(Math.toRadians(ang0));

        y11 = y01 + yL4 + c4y;
        x11 = Ox + c4x + xL4;
        

        y0 = y11 * 0.0254;
        x0 = x11 * 0.0254;

        shooter_dist = current_distance + x0;

        calcAngle = Math.toDegrees(Math.atan(2 * ((point - y0) / shooter_dist) - Math.tan(Math.toRadians(target_hitangle))));
        v0 = (1 / Math.cos(Math.toRadians(calcAngle)) ) * Math.sqrt((g * shooter_dist) / Math.abs(Math.tan(Math.toRadians(target_hitangle)) - Math.tan(Math.toRadians(calcAngle)))) ;

        //   (1 / Math.cos(Math.toRadians(calcAngle)) ) * Math.sqrt(    (g * shooter_dist) / Math.abs(Math.tan(Math.toRadians(target_hitangle)) - Math.tan(Math.toRadians(calcAngle)))));      )


        // ALL THIS LOKS BAD BUT I CANT TELL WHAT IT IS YET
        hTar = y0 + (shooter_dist * Math.tan(Math.toRadians(shooterAngleApprox))) - ((g * Math.pow(shooter_dist, 2)) / (2 * Math.pow(v0, 2) * Math.pow(Math.cos(Math.toRadians(shooterAngleApprox)), 2)));
                        

        shooterAngleApprox = (shooterAngleApprox + ( 2.0 * Math.abs(hTar - aimPoint) ));
hTar = y0 + (shooter_dist * Math.tan(Math.toRadians(shooterAngleApprox))) - ((g * Math.pow(shooter_dist, 2)) / (2 * Math.pow(v0, 2) * Math.pow(Math.cos(Math.toRadians(shooterAngleApprox)), 2)));

        absDiff = Math.abs(hTar - aimPoint);
        absInputAngle = Math.abs(inputAngle);



    }

      System.out.println("hTar " + hTar);



    // Declare variables
   double inputAngleReal = getInputAngle(shooterAngleApprox, place);

// Call getShooterAngle method
 angles = getShooterAngle(inputAngleReal);
ang0 = angles[0];
phi = angles[1];
gamma = angles[2];
beta = angles[3];


// Calculate shooter position based on input angle
if (inputAngleReal > 0) {
    a4 = 360 - beta - (180 - inputAngleReal) - gamma + psi;
    xL4 = -1 * Math.cos(Math.toRadians(a4)) * l4;
    yL4 = Math.sin(Math.toRadians(a4)) * l4;
    y01 = 6.22;
    Ox = -1.25;
} 
else {
    y01 = 7.22;
    Ox = -10.25;
    x111 = l1 * Math.cos(Math.toRadians(psi));
    y111 = l1 * -1 * Math.sin(Math.toRadians(psi));
    
    a4 = 180 - phi - gamma;
  a7 = Math.toDegrees(Math.asin(h / l1));
  a8 = a4 - psi + a7;
    
     l7x = -1 * Math.cos(Math.toRadians(a8)) * l4;
     l7y = Math.sin(Math.toRadians(a8)) * l4;
    
     a5 = 180 + inputAngleReal;
    
    xL4 = l7x + x111;
    yL4 = l7y + y111;
}

 c4 = 3.25;
 c4x = c4 * Math.cos(Math.toRadians(ang0));
 c4y = c4 * Math.sin(Math.toRadians(ang0));

 y11 = y01 + yL4 + c4y; // in inches
 x11 = Ox + c4x + xL4;

 y0 = y11 * 0.0254; // convert to meters
 x0 = x11 * 0.0254; // convert to meters

      System.out.println("y0 " + y0);

 shooter_dist = current_distance + x0;
       System.out.println("x0 " + x0);

 point = aimPoint;

     System.out.println("shooter_dist " + shooter_dist);


 return  inputAngleReal ;

}

private static double getInputAngle(double ShooterAngle , double PlaceHolder) {
    // Implement your logic here double shooterAngleApprox, double place

 // Angle Setup
 double l1 = 9.055662;; // in
 double l2 = 7.0; // in
 double l3 = 8.0; // in
 double l4 = 11.0; // in
 double h = 1.0025; // in

 // height difference between linkages
 double psi = 90.0 - Math.toDegrees(Math.acos(h / l1));

 // Array Stuff
 double shooterAngle = ShooterAngle; // the shooter angle we need, MIN value is 20 something.  Really weird behavior around 32 degrees (zero)
 double currentInputAngle = 70 * Math.signum(PlaceHolder); // the current position on the input linkage

 double guessAngle = currentInputAngle; // the "guess" angle for the arm
 double inputAngle = guessAngle; // the "guess" angle for the arm

 double shooterAngleApprox = 10.0; // this is just a variable declaration meant to be overwritten
 double beta = 0.0;

 // Angle Equations
  while (Math.abs(shooterAngle - shooterAngleApprox) >= 0.001 && Math.abs(inputAngle) < 360.0) {
     double alpha = 180.0 - inputAngle;
     double l0 = Math.sqrt(Math.pow(l2, 2) + Math.pow(l1, 2) - (2 * l1 * l2 * Math.cos(Math.toRadians(alpha))));
                       //      System.out.println("l0: " + l0);

     double phi = Math.toDegrees(Math.acos((Math.pow(l0, 2) + Math.pow(l3, 2) - Math.pow(l4, 2)) / (2 * l0 * l3)));
     double gamma = Math.toDegrees(Math.acos((Math.pow(l3, 2) + Math.pow(l4, 2) - Math.pow(l0, 2)) / (2 * l3 * l4)));
//     System.out.println("variables " + alpha + ", " + l0 + ", " + phi + ", " + gamma );
     // DISCRETE FUNCTION
     if (alpha < 180 && alpha > 0.0)  { //TODO: fix 0! Its a real issue!
         // beta = phi + acosd((l2.^2 + l0.^2 - l1.^2) / (2 * l2 * l0));
         beta = phi + Math.toDegrees(  Math.acos((Math.pow(l2, 2) + Math.pow(l0, 2) - Math.pow(l1, 2)) / (2 * l2 * l0))  );
     } else if (alpha <= 0.0 || alpha >= 180) {
         // beta = phi - acosd((l2.^2 + l0.^2 - l1.^2) / (2 * l2 * l0));
         beta = phi - Math.toDegrees(Math.acos((Math.pow(l2, 2) + Math.pow(l0, 2) - Math.pow(l1, 2)) / (2 * l2 * l0)));
     }
                                    //     System.out.println("beta: " + beta);

 //    System.out.println("Beta: " + beta);
     // FINAL SHOOTER ANGLE
     double theta = 180.0 - beta - alpha;
     shooterAngleApprox = -1.0 * (theta + psi);

     // Numerical Solver Portion
      if (guessAngle >= 49.5) {
         inputAngle = inputAngle + 0.001 * Math.abs(shooterAngle - shooterAngleApprox);
      } else if (guessAngle < 49.5) { 
         inputAngle = inputAngle - 0.001 *Math.abs(shooterAngle - shooterAngleApprox);
      }

     if (Math.abs(inputAngle) > 164) {
                     guessAngle = -1 * guessAngle;
     } 

 }
                
                return inputAngle;
                }

private static double[] getShooterAngle(double Angle) {

    double inputAngle = Angle;
    double l1 = 9.055662;
    double l2 = 7.0;
    double l3 = 8.0;
    double l4 = 11.0;
    double h = 1.0025;
    double beta = 0;

    // EQUATIONS
    // psi = 90 - acosd((h./l1));
    double psi = 90 - Math.toDegrees(Math.acos(h / l1));

    // alpha = 180 - inputAngle;
    double alpha = 180 - inputAngle;

    // l0 = sqrt(l2.^2 + l1.^2 - (2.*l1.*l2*cosd(alpha)));
    double l0 = Math.sqrt(Math.pow(l2, 2) + Math.pow(l1, 2) - (2 * l1 * l2 * Math.cos(Math.toRadians(alpha))));

    // phi = acosd((l0.^2 + l3.^2 - l4.^2) / (2 * l0 * l3));
    double phi = Math.toDegrees(Math.acos((Math.pow(l0, 2) + Math.pow(l3, 2) - Math.pow(l4, 2)) / (2 * l0 * l3)));

    // gamma = acosd((l3.^2 + l4.^2 - l0.^2) / (2 * l3 * l4));
    double gamma = Math.toDegrees(Math.acos((Math.pow(l3, 2) + Math.pow(l4, 2) - Math.pow(l0, 2)) / (2 * l3 * l4)));

    // DISCRETE FUNCTION
    if (alpha < 180 && alpha > 0) {
        // beta = phi + acosd((l2.^2 + l0.^2 - l1.^2) / (2 * l2 * l0));
        beta = phi + Math.toDegrees(Math.acos((Math.pow(l2, 2) + Math.pow(l0, 2) - Math.pow(l1, 2)) / (2 * l2 * l0)));
    } else if (alpha <= 0 || alpha >= 180) {
        // beta = phi - acosd((l2.^2 + l0.^2 - l1.^2) / (2 * l2 * l0));
        beta = phi - Math.toDegrees(Math.acos((Math.pow(l2, 2) + Math.pow(l0, 2) - Math.pow(l1, 2)) / (2 * l2 * l0)));
    }

    // FINAL SHOOTER ANGLE
    // theta = 180 - beta - alpha;
    double theta = 180 - beta - alpha;

    // shooterAngle = -1 * (theta + psi);
    double shooterAngle = -1 * (theta + psi);

    // Ensure shooterAngle is real
    shooterAngle = Math.abs(shooterAngle);



    double[] ang12 = {shooterAngle, phi, gamma, beta};
    return ang12 ;
}



public static void main(String[] args) {

double thisIsTheOne = getAimBotAngle(1.7 ,0.0, 30) ;
    System.out.println("inputAngle WOAH " + thisIsTheOne);

}




}
