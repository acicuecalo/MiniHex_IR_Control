//***********************************************************************
// Ripple Gait
// Left legs move forward rear-to-front while right also do the same,
// but right side is offset so RR starts midway through the LM stroke
//***********************************************************************
void ripple_gait()
{

  //read commanded values from controller
  commandedX = map(RY_Smoothed, 255, 0, 127, -127);
  commandedY = map(RX_Smoothed, 0, 255, -127, 127);
  commandedR = map(LX_Smoothed, 0, 255, 127, -127);


  //if commands more than deadband then process
  if ((abs(commandedX) > DEATHBAND) || (abs(commandedY) > DEATHBAND) || (abs(commandedR) > DEATHBAND) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); //total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (ripple_case[leg_num])
      {
        case 1:                               //move foot forward (raise)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / (numTicks * 2));
          if (tick >= numTicks - 1) ripple_case[leg_num] = 2;
          break;
        case 2:                               //move foot forward (lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * (numTicks + tick) / (numTicks * 2));
          if (tick >= numTicks - 1) ripple_case[leg_num] = 3;
          break;
        case 3:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 4;
          break;
        case 4:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 5;
          break;
        case 5:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 6;
          break;
        case 6:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}
