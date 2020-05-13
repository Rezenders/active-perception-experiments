search_area(25).
flight_altitude(5).
setpoint_goal(0,0,0).
!plan_path.

+!plan_path: local_pos(X1,Y1,Z1,X2,Y2,Z2,W2)
	<- 	?search_area(A);
			plan_path(X1,Y1,Z1,X2,Y2,Z2,W2,[[X1-A,Y1,0],[X1-A,Y1+A,0],[X1+A,Y1+A,0],[X1+A,Y1,0]]).

+!plan_path <- !plan_path.

+plan_path_result(PLIST)
	<-
			!!publishSetPoint;
			.wait(1000);
			!armMotor;
			set_mode("OFFBOARD");
			!define_goal(PLIST).

+!define_goal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
			-+setpoint_goal(X,Y,Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.5 & math.abs(Y2 -(Y)) <=0.5 & math.abs(Z2 -(Z)) <=0.5);
			!define_goal(T).

+!define_goal([])<- .drop_intention(publishSetPoint).

+!publishSetPoint
	<-	?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(200);
			!publishSetPoint.

+!armMotor : not state(_,_,"True")
	<-	arm_motors(True);
			.wait(state(_,_,"True"), 1000).

+!armMotor.

-!armMotor <- !armMotor.
