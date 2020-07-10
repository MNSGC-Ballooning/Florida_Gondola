/*void sendConfirmation(){
      // checksum = MTcdu1Packet_tx[0] + MTcdu1Packet_tx[1] + MTcdu1Packet_tx[2];
      // MTcdu1Packet_tx[3] = checksum >> 8;
      // MTcdu1Packet_tx[4] = checksum;
      Serial4.write(MTcdu1Packet_tx, CDU_TX_SIZE);

      //cdu1_watchdog = millis();
      /*Serial.println();
      for (int i=0;i<6;i++){
        Serial.print(MTcdu1Packet_tx[i],HEX);
        Serial.print(" ");
      }
      Serial.println();//for printing packet to serial for debugging
            
      checksum = MTcdu2Packet_tx[0] + MTcdu2Packet_tx[1] + MTcdu1Packet_tx[2];
      MTcdu2Packet_tx[3] = checksum >> 8;
      MTcdu2Packet_tx[4] = checksum;
      Serial5.write(MTcdu2Packet_tx, CDU_TX_SIZE);
      //cdu2_watchdog = millis();
      checksum = 0;
}*/
