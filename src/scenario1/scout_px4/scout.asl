/* Initial beliefs and rules */
water_y_offset(12.5).
search_area(12).
flight_altitude(3).
setpoint_goal(0,0,0).

/* Initial goals */
!setRTLAtlitude(5).
!setMaxSpeed(3).
!planPath.

/* Plans */
+!setMaxSpeed(S)
	<- set_fcu_param("MPC_XY_VEL_MAX", 0, S).

+!setRTLAtlitude(A)
	<- 	set_fcu_param("RTL_RETURN_ALT", 0, A).

+!planPath: local_pos(X1,Y1,Z1,X2,Y2,Z2,W2)
	<- 	?search_area(A);?water_y_offset(Y_OFFSET);
			plan_path(X1,Y1,Z1,X2,Y2,Z2,W2,[[X1-A, Y1+Y_OFFSET, 0],[X1-A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+Y_OFFSET, 0]]).

+!planPath <- !planPath.

+plan_path_result(PLIST)
	<-	camera_switch(True);
			!!publishSetPoint;
			.wait(2000);
			+mode("Fly");
			!!contactRescuers;
			!defineGoal(PLIST).

/* Define setpoint goal*/
+!defineGoal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
			-+setpoint_goal(X,Y,Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.5 & math.abs(Y2 -(Y)) <=0.5 & math.abs(Z2 -(Z)) <=0.5);
			!defineGoal(T).

+!defineGoal([])
	<-	.drop_intention(publishSetPoint).

+!publishSetPoint : (mode("Fly") & state("OFFBOARD",_,"True")) | (not mode("Fly"))
	<-	?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(100);
			!publishSetPoint.

+!publishSetPoint : mode("Fly") & not state("OFFBOARD",_,"True")
	<-	arm_motors(True);
			set_mode("OFFBOARD");
			?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(100);
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
			.broadcast(tell, victim_in_need(ID, GX, GY)[lu(HH,MM,SS,MS)]);
			-victim_position(ID, _, _);
			.wait(500);
			!informVictim(T).

+!informVictim([]).

+!mark_as_rescued(_, _, _).
