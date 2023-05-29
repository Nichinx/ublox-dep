void send_rtcm(){
	  uint8_t buf[BUFLEN];
	  uint8_t* bufptr;
	  int bytesRead;
	  int bytesLeft;
	  if (U_SERIAL.available())
	  {
	    digitalWrite(LED, HIGH);
	    bytesRead = U_SERIAL.readBytes((char *) buf, BUFLEN); // If timeout is set properly, this should read an entire burst from the M8P.
	    Serial.println(bytesRead);  //This is useful for debugging if the serial monitor is open. It doesn't add much overhead to the program.

	    // Process the entire received buffer (buf) in individual 200 byte packets.
	    // bufptr points to the start of the bytes to be transmitted. It is moved through buf until all bytes are transmitted.
	    bufptr = buf;
	    bytesLeft = bytesRead;
	    while(bytesLeft > 0) {
		      if( bytesLeft < 200)
		      {
		        rf95.send(bufptr, bytesLeft);
		        bytesLeft = 0;
		      }
		      else
		      {
		        rf95.send(bufptr, 200);
		        bufptr += 200;
		        bytesLeft -= 200;
		      }
	    }
	   } else {
	   	Serial.println("no rtcm");
	   } 
	  digitalWrite(LED, LOW);
}
