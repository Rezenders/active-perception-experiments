/* Initial beliefs and rules */
water_y_offset(2.5).
search_area(13).
flight_altitude(3).
setpoint_goal(0,0,0).

/* Initial goals */
!setRTLAtlitude(5.0).
!setMaxSpeed(6).
!planPath.
!inRange.

/* Plans */
+!setMaxSpeed(S)
	<- 	set_fcu_param("MPC_XY_VEL_MAX", 0, S).

+!setRTLAtlitude(A)
	<- 	set_fcu_param("RTL_RETURN_ALT", 0, A).

+!planPath: local_pos(X1,Y1,Z1,X2,Y2,Z2,W2)
	<- 	?search_area(A);?water_y_offset(Y_OFFSET);
			plan_path(X1,Y1,Z1,X2,Y2,Z2,W2,[[X1-A, Y1+Y_OFFSET, 0],[X1-A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+Y_OFFSET, 0]]).

+!planPath <- !planPath.

+!inRange
	<-	.time(HH,MM,SS,MS);
			+range[ap(_),lu(HH,MM,SS,MS)].

+plan_path_result(PLIST)
	<-	camera_switch(True);
			!!publishSetPoint;
			.wait(2000);
			!!contactRescuers;
			!defineGoal(PLIST).

/* Define setpoint goal*/
+!defineGoal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
			-+setpoint_goal(X,Y,Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.7 & math.abs(Y2 -(Y)) <=0.7 & math.abs(Z2 -(Z)) <=0.7);
			!defineGoal(T).

+!defineGoal([])
	<-	.drop_intention(publishSetPoint).

+!publishSetPoint : state("OFFBOARD",_,"True")
	<-	?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(200);
			!publishSetPoint.

+!publishSetPoint : not state("OFFBOARD",_,"True")
	<-	arm_motors(True);
			set_mode("OFFBOARD");
			?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(200);
			!publishSetPoint.

+victim(ID, GX, GY)
	<-	+victim_position(ID, GX, GY).

+victim_position(ID, GX, GY)
	<-	.resume(contactRescuers).

+!contactRescuers: victim_position(_, _, _)
 	<- 	.findall([ID, GX, GY], victim_position(ID, GX, GY), VLIST);
			!informVictim(VLIST);
			!contactRescuers.

+!contactRescuers <- .suspend; !contactRescuers.

+!informVictim([H|T])
	<- 	 H = [ID, GX, GY]
			.print("Victim ", H);
			.time(HH,MM,SS,MS);
			// .broadcast(tell, victim_in_need(ID, GX, GY)[lu(HH,MM,SS,MS)]);
			!broadcast(tell, victim_in_need(ID, GX, GY)[lu(HH,MM,SS,MS)]);
			-victim_position(ID, _, _);
			.wait(500);
			!informVictim(T).

+!informVictim([]).

+!broadcast(Itl, Data) : range[ap(1000)]
	<- .broadcast(Itl, Data);
			.print("Sending MSG!!!!!!!").

+!broadcast(Itl, Data).

+?range[ap]
	<-	.print("Flying to comm range!");
			.suspend(defineGoal(_));
			?local_pos(LX, LY, LZ, _, _, _, _);
			?setpoint_goal(NX, NY, NZ);
			X=0;Y=4;
			?flight_altitude(Z);
			-+setpoint_goal(X, Y, Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.7 & math.abs(Y2 -(Y)) <=0.7 & math.abs(Z2 -(Z)) <=0.7);
			.time(HH,MM,SS,MS);
			+range[ap(_),lu(HH,MM,SS,MS)];
			!!returntToPath([LX,LY,LZ],[NX,NY,NZ]).

+!returntToPath(H,T)
	<-	H = [X1, Y1, Z1];
			-+setpoint_goal(X1, Y1, Z1);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X1)) <=0.7 & math.abs(Y2 -(Y1)) <=0.7 & math.abs(Z2 -(Z1)) <=0.7);
			T = [X3, Y3, Z3];
			-+setpoint_goal(X3, Y3, Z3);
			.wait(local_pos(X4,Y4,Z4,_,_,_,_) & math.abs(X4 -(X3)) <=0.7 & math.abs(Y4 -(Y3)) <=0.7 & math.abs(Z4 -(Z3)) <=0.7);
			.resume(defineGoal(_)).

+!mark_as_rescued(N, Lat, Long).

{apply_ap}
