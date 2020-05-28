water_y_offset(12.5).
search_area(10).
flight_altitude(3).
setpoint_goal(0,0,0).
!setMaxSpeed(3).
!planPath.

+!setMaxSpeed(S)
	<- set_fcu_param("MPC_XY_VEL_MAX", 0, S).

+!planPath: local_pos(X1,Y1,Z1,X2,Y2,Z2,W2)
	<- 	?search_area(A);?water_y_offset(Y_OFFSET);
			plan_path(X1,Y1,Z1,X2,Y2,Z2,W2,[[X1-A, Y1+Y_OFFSET, 0],[X1-A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+A+Y_OFFSET, 0],[X1+A, Y1+Y_OFFSET, 0]]).

+!planPath <- !planPath.

+plan_path_result(PLIST)
	<-
			!!publishSetPoint;
			.wait(1000);
			!setMode("OFFBOARD");
			!armMotor;
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
	<- 	?global_pos(Lat, Long);
			?local_pos(X, Y,_,_,_,_,_);
			-victim(ID,_,_,_,_,_);
			+victim(ID, Score, Lat, Long, X, Y).

@addVictim2[atomic]
+!addVictim(ID, Score): not victim(ID,_,_,_,_,_)
	<- 	?global_pos(Lat, Long);
			?local_pos(X, Y,_,_,_,_,_);
			+victim(ID, Score, Lat, Long, X, Y).

@addVictim3[atomic]
+!addVictim(ID, Score).

+!defineGoal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
			-+setpoint_goal(X,Y,Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.5 & math.abs(Y2 -(Y)) <=0.5 & math.abs(Z2 -(Z)) <=0.5);
			!defineGoal(T).

+!defineGoal([])<- .drop_intention(publishSetPoint).

+!publishSetPoint
	<-	?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(100);
			!publishSetPoint.

+!armMotor : not state(_,_,"True")
	<-	arm_motors(True);
			.wait(state(_,_,"True"), 1000).

+!armMotor.

-!armMotor <- !armMotor.

+!setMode(Mode) : not state(Mode,_,_)
	<- 	!armMotor;
			set_mode(Mode);
			.wait(state(Mode,_,_), 1000).

+!setMode(Mode).

-!setMode(Mode) <- !setMode(Mode).
