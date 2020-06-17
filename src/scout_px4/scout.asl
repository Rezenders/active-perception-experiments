water_y_offset(12.5).
search_area(10).
flight_altitude(3).
setpoint_goal(0,0,0).

!setRTLAtlitude(5).
!setMaxSpeed(3).
!planPath.

+!setMaxSpeed(S)
	<- set_fcu_param("MPC_XY_VEL_MAX", 0, S).

+!setRTLAtlitude(A)
	<- 	set_fcu_param("RTL_RETURN_ALT", 0, A).

+!planPath: local_pos(X1,Y1,Z1,X2,Y2,Z2,W2)
	<- 	?search_area(A);?water_y_offset(Y_OFFSET);
			plan_path(X1,Y1,Z1,X2,Y2,Z2,W2,[[X1-A, Y1+Y_OFFSET, 0],[X1-A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+Y_OFFSET, 0]]).

+!planPath <- !planPath.

+plan_path_result(PLIST)
	<-	!!publishSetPoint;
			.wait(2000);
			+mode("Fly");
			camera_switch(True);
			!!contactRescuers;
			!defineGoal(PLIST).

+object_tracker(OBJ_LIST)
	<- !getObjects(OBJ_LIST).

+!getObjects([H|T])
	<-	H = [Label, ID, Score];
			.term2string(Label, LabelStr);
			if(LabelStr == "victim"){
				!addVictim(ID, Score);
			}
			!getObjects(T).

+!getObjects([]).

@addVictim[atomic]
+!addVictim(ID, Score) : victim(ID, S, _, _, _, _) & Score > S
	<- 	?global_pos(Lat, Long, _);
			?local_pos(X, Y,_,_,_,_,_);
			-victim(ID,_,_,_,_,_);
			+victim(ID, Score, Lat, Long, X, Y).

@addVictim2[atomic]
+!addVictim(ID, Score): not victim(ID,_,_,_,_,_)
	<- 	?global_pos(Lat, Long, _);
			?local_pos(X, Y,_,_,_,_,_);
			+victim(ID, Score, Lat, Long, X, Y).

+!addVictim(ID, Score).

+victim(ID, Score, Lat, Long, X, Y)
	<-	.resume(contactRescuers).

+!contactRescuers: victim(_,_,_,_,_,_)
 	<- 	.wait(5000);
			.findall([ID, Score, Lat, Long, X, Y], victim(ID, Score, Lat, Long, X, Y), VLIST);
			!informVictim(VLIST);
			!contactRescuers.

+!contactRescuers <- .suspend; !contactRescuers.

+!informVictim([H|T])
	<- 	 H = [ID, Score, Lat, Long, X, Y]
			.print(H);
			.broadcast(tell, victim_in_need(ID, Lat, Long));
			-victim(ID, Score, Lat, Long, X, Y);
			.wait(500);
			!informVictim(T).

+!informVictim([]).

+!defineGoal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
			-+setpoint_goal(X,Y,Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.5 & math.abs(Y2 -(Y)) <=0.5 & math.abs(Z2 -(Z)) <=0.5);
			!defineGoal(T).

+!defineGoal([])
	<-	camera_switch(False);
			.drop_intention(publishSetPoint).

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

+!armMotor : not state(_,_,"True")
	<-	arm_motors(True);
			.wait(state(_,_,"True"), 500).

+!armMotor.

-!armMotor <- !armMotor.

+!setMode(Mode) : not state(Mode,_,_)
	<- 	set_mode(Mode);
			.wait(state(Mode,_,_), 200).

+!setMode(Mode).

-!setMode(Mode) <- !setMode(Mode).

+!mark_as_rescued(N, Lat, Long).
