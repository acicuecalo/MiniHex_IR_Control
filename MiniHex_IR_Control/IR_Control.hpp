void Setup_IR_Receiver() {
  //Start the receiver, enable feedback LED and take LED feedback pin from the internal boards definition
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN);
  Serial.print(F("Ready to receive IR signals at pin "));
  Serial.println(IR_RECEIVE_PIN); // Pin D2 on Hexpider
}

void Decode_IR_Receiver() {
  if (IrReceiver.decode()) {   //Check if received data is available and if yes, try to decode it
    //PrintSummary();
    IrReceiver.resume(); // Enable receiving of the next value

    //Check the received data and perform actions according to the received command
    if (IrReceiver.decodedIRData.command == 0x18 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("Up");
      RY = 127 + 100; // Fordward movement
      LX = 127; // Center LX
      HX = 90;  // Center head pan
      HY = 90 - 20; // Down head tilt
    }
    else if (IrReceiver.decodedIRData.command == 0x52 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("Down");
      RY = 127 - 100; // Backward movement
      LX = 127; // Center LX
      HX = 90;  // Center head pan
      HY = 90 + 45; // Down head tilt
    }
    else if (IrReceiver.decodedIRData.command == 0x8 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("Left");
      RX = 127 - 100; // Left side movement
      HX = 50;  // Left head pan rotation
    }
    else if (IrReceiver.decodedIRData.command == 0x5A && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("Right");
      RX = 127 + 100; // Right side movement
      HX = 130; // Right head pan
    }
    else if (IrReceiver.decodedIRData.command == 0x1C && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("OK");
      //Stop robot and center head
      RX = 127;
      RY = 127;
      LX = 127;
      HX = 90; // Center head pan
      HY = 90;  // Center head tilt
    }
    else if (IrReceiver.decodedIRData.command == 0x16 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("*");
      LX = 77; // Left rotation movement
      HX = 40;//  Left head pan
    }
    else if (IrReceiver.decodedIRData.command == 0xD && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("#");
      LX = 177; // Right rotation movement
      HX = 140;//  Right head pan
    }
    else if (IrReceiver.decodedIRData.command == 0x45 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("1");
      mode = 1;
    }
    else if (IrReceiver.decodedIRData.command == 0x46 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("2");
      mode = 2;
    }
    else if (IrReceiver.decodedIRData.command == 0x47 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("3");
      mode = 3;
    }
    else if (IrReceiver.decodedIRData.command == 0x44 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("4");
      gait = 0;
    }
    else if (IrReceiver.decodedIRData.command == 0x40 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("5");
      gait = 1;
    }
    else if (IrReceiver.decodedIRData.command == 0x43 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("6");
      gait = 2;
    }
    else if (IrReceiver.decodedIRData.command == 0x7 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("7");
      gait = 3;
    }
    else if (IrReceiver.decodedIRData.command == 0x15 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("8");
    }
    else if (IrReceiver.decodedIRData.command == 0x9 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("9");
    }
    else if (IrReceiver.decodedIRData.command == 0x19 && IrReceiver.decodedIRData.address == 0x0) {
      //Serial.println("0");
    }
    else {
      //Serial.println("Not set yet");
    }
  }
}
