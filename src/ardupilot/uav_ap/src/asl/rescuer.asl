!start_rescue.

+!start_rescue
	<- 	.time(HH,MM,SS,MS);
			+victim(-27.603683, -48.518052)[ap(1000),lu(HH,MM,SS,MS)];
			!rescueVictim.

+!rescueVictim : victim(Lat, Long)
	<-	.print("Starting Jason Agent node.");
			.wait(state(_,"True",_));
			!setMode("GUIDED");
			!armMotor;
			!takeOff(5);
			!goToPos(Lat, Long, 40);
			!dropBuoy(Lat, Long);
			!returnToLand.

+!dropBuoy(Lat, Long) : victim(Lat, Long)[ap(1000)]
	<- .print("Droping Buoy").

+!dropBuoy(Lat, Long)
	<- .print("No victim to drop Buoy").

+?victim(Lat, Long)[ap(T)]
	<- 	.print("active perception victim");
			Delta = 0.0001;
			!goToPos(Lat + Delta, Long + Delta, 50);
			!goToPos(Lat + Delta, Long - Delta, 50);
			!goToPos(Lat - Delta, Long - Delta, 50);
			!goToPos(Lat - Delta, Long + Delta, 50);
			!goToPos(Lat + Delta, Long + Delta, 50);
			.time(HH,MM,SS,MS);
			+victim(Lat, Long)[ap(T),lu(HH,MM,SS,MS)].


+!setMode(Mode) : not state(Mode,_,_)
	<- 	set_mode(Mode);
			.wait(state(Mode,_,_), 1000).

+!setMode(Mode).

-!setMode(Mode) <- !setMode(Mode).

+!armMotor : not state(Mode,_,"True")
	<-	arm_motors(True);
			.wait(state(Mode,_,"True"), 1000).

+!armMotor.

-!armMotor <- !armMotor.

+!takeOff(Alt)
	<-	takeoff(Alt);
			.wait(altitude(A) & math.abs(A-Alt) <= 0.1).

+!goToPos(Lat, Long, Alt)
	<- 	setpoint(Lat, Long, Alt);
			.wait(global_pos(X,Y) & math.abs(X -(Lat)) <=0.00001 & math.abs(Y -(Long)) <=0.00001).

+!returnToLand
	<-	set_mode("RTL");
			.wait(global_pos(X,Y) & home_pos(X2,Y2) & math.abs(X -(X2)) <=0.00001 & math.abs(Y -(Y2)) <=0.00001 & altitude(A) & math.abs(A-0) <= 0.1).

{apply_ap}
