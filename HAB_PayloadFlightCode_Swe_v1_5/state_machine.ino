/////////////////////////////////////////////////////////////////////////////////////////////////
///////////******************* BEGINNING OF STATE MACHINE FUNCTIONS ***************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

void stateMachine() {
  // ensure the state machine does not start until a certain intial altitude is reached
  static bool initDone = false;
  static byte initCounter = 0;
  if(!initDone && (stat > 7)) { 
    state = INITIALIZATION;
    stateString = F("Initialization");
    if(Altitude[0] > INIT_ALTITUDE) {
      initCounter++;
      if(initCounter >= 10) {
        initCounter = 0;
        initDone = true; 
      }
    }
  }

  // run state switch function if the state machine is intialized
  if (initDone) { 
    CompareGPS();
    stateSwitch();
  }

  uint16_t checksum;  
  // run functions based off of the current state
  switch(state) {
    ///// Ascent /////
    case 0x01:
      stateString = F("Ascent");

      static unsigned long ascentStamp = millis();

      // cut balloon A if the ascent timer runs out
      if(millis() - ascentStamp > ASCENT_INTERVAL*60000) {
        //******************* SEND CUT COMMAND TO CUTTER COMPUTER A ******************////////////
        checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
        cdu1Packet_tx[3] = checksum >> 8;
        cdu1Packet_tx[4] = checksum;
        Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
        cutReasonA = F("expired ascent timer");
      }
      // cut balloon A if the termination altitude is reached
      if (Altitude[0] > SLOW_DESCENT_CEILING) {
        //******************* SEND CUT COMMAND TO CUTTER COMPUTER A *********************////////////
        checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
        cdu1Packet_tx[3] = checksum >> 8;
        cdu1Packet_tx[4] = checksum;
        Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
        cutReasonA = F("reached termination altitude");
      }
      else cutReasonA = F("0");

      break;

    ///// Slow Ascent /////
    case 0x02:
      // cut the resistor and note state
      stateString = F("Slow Ascent");

      // cut both balloons as the stack is ascending too slowly
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
      checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
      cdu1Packet_tx[3] = checksum >> 8;
      cdu1Packet_tx[4] = checksum;
      Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
      cutReasonA = F("slow ascent state");
      
      checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2];
      cdu2Packet_tx[3] = checksum >> 8;
      cdu2Packet_tx[4] = checksum;
      Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);
      cutReasonB = F("slow ascent state");

      break;

    ///// Slow Descent /////      
    case 0x04:
      // organize timing schema for slow descent state
      stateString = F("Slow Descent");

      static unsigned long slowDescentStamp = millis(); // initializaed upon first time in this state
      static byte SDTerminationCounter = 0;

      if(millis() - slowDescentStamp > SLOW_DESCENT_INTERVAL*60000 || (Altitude[0] < SLOW_DESCENT_FLOOR && Altitude[0] != 0)) {
        SDTerminationCounter++;

        if(SDTerminationCounter >= 10) {
          //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ******************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
          cdu1Packet_tx[3] = checksum >> 8;
          cdu1Packet_tx[4] = checksum;
          Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
          cutReasonA = F("reached slow descent floor");

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2];
          cdu2Packet_tx[3] = checksum >> 8;
          cdu2Packet_tx[4] = checksum;
          Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);
          cutReasonB = F("reached slow descent floor");
        }
      }

      break;

    ///// Descent /////
    case 0x08:
      // do nothing but note state
      stateString = F("Descent");

      static byte floorAltitudeCounter = 0;   // increments if the stack is below the slow descent altitude floor
      static bool cutCheck = false;

      if(Altitude[0] < SLOW_DESCENT_FLOOR && Altitude[0] != 0) {
        floorAltitudeCounter++;

        if(floorAltitudeCounter >= 10 && !cutCheck) {
          floorAltitudeCounter = 0;

          //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B *******************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
          cdu1Packet_tx[3] = checksum >> 8;
          cdu1Packet_tx[4] = checksum;
          Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
          cutReasonA = F("descent state cut check");

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2];
          cdu2Packet_tx[3] = checksum >> 8;
          cdu2Packet_tx[4] = checksum;
          Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);    
          cutReasonB = F("descent state cut check");
        }
      }

      break;

    ///// Float /////
    case 0x10:
      // abort flight
      stateString = F("Float");

      // cut both balloons as the stack is in a float state
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
      checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
      cdu1Packet_tx[3] = checksum >> 8;
      cdu1Packet_tx[4] = checksum;
      Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
      cutReasonA = F("float state");
      
      checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2];
      cdu2Packet_tx[3] = checksum >> 8;
      cdu2Packet_tx[4] = checksum;
      Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);    
      cutReasonB = F("float state");

      break;

    ///// Out of Boundary /////
    case 0x20:
      // cut resistor and note state
      stateString = F("Out of Boundary");
      
      // cut both balloons as the stack is out of the predefined flight boundaries
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
      checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
      cdu1Packet_tx[3] = checksum >> 8;
      cdu1Packet_tx[4] = checksum;
      Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
      
      checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2];
      cdu2Packet_tx[3] = checksum >> 8;
      cdu2Packet_tx[4] = checksum;
      Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);    
      // cut reasons are more specifically defined in the boundaryCheck() function

      break;

    ///// Temperature Failure /////
    case 0x40:
      // cut resistor and note state
      stateString = F("Temperature Failure");

      // cut balloon as temps are at critical levels
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
      checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2];
      cdu1Packet_tx[3] = checksum >> 8;
      cdu1Packet_tx[4] = checksum;
      Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
      
      checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2];
      cdu2Packet_tx[3] = checksum >> 8;
      cdu2Packet_tx[4] = checksum;
      Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);
      
      cutReasonA = F("Temperature failure");
      cutReasonB = F("Temperature failure");

      break;

    ///// Recovery /////
    case 0x80:
      // reserved for any functions near the ground
      stateString = F("Recovery");

      break;

  }
  
}


void stateSwitch() {
  // initialize all counters as static bytes that begin at zero
  static byte ascentCounter = 0,  slowAscentCounter = 0,  descentCounter = 0, slowDescentCounter = 0, floatCounter = 0, recoveryCounter = 0, boundaryCounter = 0, tempCounter = 0;

  if(stateSwitched) {     // reset all state counters if the state was just switched
    ascentCounter = 0;  slowAscentCounter = 0;  descentCounter  = 0;  slowDescentCounter = 0;   floatCounter  = 0; recoveryCounter = 0;  boundaryCounter = 0, tempCounter = 0;
    stateSwitched = false;
  }

  if(ascentRate > MAX_SA_RATE && state != ASCENT) {
    ascentCounter++;
    if(ascentCounter >= 40) {
      state = ASCENT;
      ascentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate <= MAX_SA_RATE && ascentRate > MAX_FLOAT_RATE && state != SLOW_ASCENT) {
    slowAscentCounter++;
    if(slowAscentCounter >= 40) {
      state = SLOW_ASCENT;
      slowAscentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate <= MAX_FLOAT_RATE && ascentRate >= MIN_FLOAT_RATE && state != FLOAT) {
    floatCounter++;
    if(floatCounter >= 600) {
      state = FLOAT;
      floatCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate < MIN_FLOAT_RATE && ascentRate >= MIN_SD_RATE && state != SLOW_DESCENT) {
    slowDescentCounter++;
    if(slowDescentCounter >= 40) {
      state = SLOW_DESCENT;
      slowDescentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate < MIN_SD_RATE && state !=DESCENT) {
    descentCounter++;
    if(descentCounter >= 40) {
      state = DESCENT;
      descentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(state != RECOVERY && (state == DESCENT || state == SLOW_DESCENT) && Altitude[0] < RECOVERY_ALTITUDE) {
    recoveryCounter++;
    if(recoveryCounter >= 40) {
      state = RECOVERY;
      recoveryCounter = 0;
      stateSwitched = true;
    }
  }

  // part of a separate series of if/else statements as criteria for this state is different
  if(boundaryCheck() && state != OUT_OF_BOUNDARY) {
    boundaryCounter++;
    if(boundaryCounter >= 40) {
      state = OUT_OF_BOUNDARY;
      boundaryCounter = 0;
      stateSwitched = true;
    }
  } 

  if(!(MIN_TEMP<tempInt_float<MAX_TEMP) && state != TEMPERATURE_FAILURE)  {
    tempCounter++;
    if(tempCounter >= 40) {
      state = TEMPERATURE_FAILURE;
      tempCounter  = 0;
      stateSwitched = true;
    }
  }  
}

bool boundaryCheck() {
  // function to check if the payload is out of the flight boundaries
  if (longitude[0] > EASTERN_BOUNDARY) {
    cutReasonA = F("reached eastern boundary");
    cutReasonB = F("reached eastern boundary");
    return true;
  }
  else if (longitude[0] < WESTERN_BOUNDARY) {
    cutReasonA = F("reached western boundary");
    cutReasonB = F("reached western boundary");
    return true;
  }
  else if (latitude[0] > NORTHERN_BOUNDARY) {
    cutReasonA = F("reached northern boundary");
    cutReasonB = F("reached northern boundary");
    return true;
  }
  else if (latitude[0] < SOUTHERN_BOUNDARY) {
    cutReasonA = F("reached southern boundary");
    cutReasonB = F("reached southern boundary");
    return true; 
  }
  else {
    return false;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////******************* END OF STATE MACHINE FUNCTIONS *********************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
