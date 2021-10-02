//***********************************************************************
// Tripod Gait
// Group of 3 legs move forward while the other 3 legs provide support
//***********************************************************************
void tripod_gait()
{
  //read commanded values from controller
  commandedX = map(RY_Smoothed, 255, 0, 127, -127);
  commandedY = map(RX_Smoothed, 0, 255, -127, 127);
  commandedR = map(LX_Smoothed, 0, 255, 127, -127);

  //if commands more than deadband then process
  if ((abs(commandedX) > DEATHBAND) || (abs(commandedY) > DEATHBAND) || (abs(commandedR) > DEATHBAND) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); //total ticks divided into the two cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tripod_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) tripod_case[leg_num] = 2;
          break;
        case 2:                               //move foot back (on the ground)
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tripod_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}
