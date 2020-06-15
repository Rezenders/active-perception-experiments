flight_altitude(2).

!setRTLAtlitude(3).
!startPublishingSetPoints.

+!setRTLAtlitude(A)
  <-  set_fcu_param("RTL_RETURN_ALT", 0, A);
      set_fcu_param("RTL_DESCEND_ALT", 0, A-0.5).

+!startPublishingSetPoints
  <-  .wait(home_pos(HLat, HLong, _));
      ?home_pos(HLat, HLong, _);
      -+setpoint_goal(HLat, HLong, 0);
      !!publishSetPoint.

+victim_in_need(N, Lat, Long)
	<-	!start_negotiation.

+!start_negotiation: .desire(negotiate) | .desire(rescueVictim(_, _, _))
	<- 	.suspend;
			!!start_negotiation.

@lg[atomic]
+!start_negotiation
	<- !negotiate.

+!negotiate
	<-	.findall([N, Lat, Long], victim_in_need(N,Lat,Long), V);
			.sort(V, SV);
			.nth(0, SV, Next);
			Next = [N, Lat, Long];

			!propose(N);
			.findall([O, Me], propose(Me, N, O), L);
			!choose_proposal(N, L).

-!negotiate.

+!propose(N)
	<- 	.random(R);
      .my_name(Me);
			+propose(Me, N, R);
			!broadcastProposal(10, Me, N, R).

+!broadcastProposal(I, Me, N, R) : I>0
  <-  .broadcast(tell, propose(Me, N, R));
      .wait(200);
      !broadcastProposal(I-1, Me, N, R).

+!broadcastProposal(I, Me, N, R).

+!choose_proposal(N, L)
	<-	.min(L, [WOffer, Who]);
			!check_winner(N, Who);
			.abolish(propose(_,_,_)).

-!choose_proposal(N, L).

+!check_winner(N, Who): .my_name(Me) & Me == Who
	<-	.print("I am responsible for rescuing victim ", N);
			?victim_in_need(N, Lat, Long);
      .abolish(victim_in_need(N, Lat, Long));
			.broadcast(achieve, mark_as_rescued(N, Lat, Long));
      .resume(publishSetPoint);
			!!rescueVictim(N, Lat, Long).

+!check_winner(N, Who)
	<- 	.print("Not selected!").

+!rescueVictim(N, Lat, Long)
	<- 	.wait(1000);
			// !armMotor;
			// !setMode("OFFBOARD");
			// !armMotor;
      +mode("Fly");
      !defineGoal([[Lat, Long,_]]);
      !getGazeboPos;
      ?gazebo_pos(X, Y, Z);
      drop_buoy(X, Y, Z-0.25);
			.print("Droping buoy to victim ", N);
			.broadcast(tell, victim_rescued(N, Lat, Long));
      !returnHome;
			.resume(start_negotiation).

+!mark_as_rescued(N, Lat, Long)
	<- .abolish(victim_in_need(N,Lat,Long)).

+!defineGoal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
      ?altitude(A);
      ?home_pos(_,_,HA);
			-+setpoint_goal(X, Y, A+Z);
			.wait(global_pos(X2,Y2,Z2) & math.abs(X2 -(X)) <=0.00001 & math.abs(Y2 -(Y)) <=0.00001 & math.abs(Z2 -(HA+Z)) <= 0.5);
			!defineGoal(T).

+!defineGoal([]).

+!publishSetPoint : (mode("Fly") & state("OFFBOARD",_,"True")) | (not mode("Fly"))
	<-	?setpoint_goal(X,Y,Z);
			setpoint_global(X,Y,Z);
			// setpoint_local(X,Y,Z);
			.wait(100);
			!publishSetPoint.

+!publishSetPoint : mode("Fly") & not state("OFFBOARD",_,"True")
	<-	arm_motors(True);
			set_mode("OFFBOARD");
			?setpoint_goal(X,Y,Z);
			setpoint_global(X,Y,Z);
			.wait(100);
			!publishSetPoint.

+!returnHome
  <-  .suspend(publishSetPoint);
      -mode("Fly");
      ?home_pos(X2,Y2,Z2);
      -+setpoint_goal(X2, Y2, Z2);
      .wait(global_pos(X,Y,Z) & math.abs(X2 -(X)) <=0.00001 & math.abs(Y2 -(Y)) <=0.00001 & math.abs(Z2 -(Z)) <= 0.1);
      .print("Landed! beginning charging and buoy replacement!");
      .wait(1000).

+!armMotor : not state(_,_,"True")
	<-	arm_motors(True);
			.wait(400);
      !armMotor.

+!armMotor.

// -!armMotor <- !armMotor.

+!setMode(Mode) : not state(Mode,_,_)
	<- 	set_mode(Mode);
      .wait(200);
      !setMode(Mode).

+!setMode(Mode).

// -!setMode(Mode) <- !setMode(Mode).

+!getGazeboPos
  <-  ?model_states(Models, Poses);
      .length(Models, Length);
      for(.range(I, 0, Length-1)){
        .nth(I, Models, Model); .term2string(Model, SModel);
        .my_name(Me); .term2string(Me, SMe);
        if(SModel == SMe){
          .abolish(gazebo_pos(_,_,_));
          .nth(I, Poses, Pose);
          Pose = [X, Y, Z];
          +gazebo_pos(X, Y, Z);
        }
      }
      .
