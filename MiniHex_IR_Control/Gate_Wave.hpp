
//***********************************************************************
// Wave Gait
// Legs move forward one at a time while the other 5 legs provide support
//***********************************************************************
void wave_gait()
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
      switch (wave_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) wave_case[leg_num] = 6;
          break;
        case 2:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 1;
          break;
        case 3:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 2;
          break;
        case 4:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1)
            wave_case[leg_num] = 3;
          break;
        case 5:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 4;
          break;
        case 6:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 5;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}
