// adj(X,Y1,X,Y2) :- Y2 == Y1-1.

!start_rescue.

+!start_rescue
	<- 	.time(HH,MM,SS,MS);
			+victim(-27.603683, -48.518052)[ap(1000),lu(HH,MM,SS,MS)];
			!rescueVictim.

+!rescueVictim : victim(Lat, Long)
	<-	.print("Starting Jason Agent node.");
			!setMode("GUIDED");
			arm_motors(True);
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
			.time(HH,MM,SS,MS);
			+victim(Lat, Long)[ap(T),lu(HH,MM,SS,MS)].


+!setMode(Mode)
	<- 	set_mode(Mode);
		.wait(state(Mode)).

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
