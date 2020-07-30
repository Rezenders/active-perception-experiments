/* Initial beliefs and rules */
flight_altitude(2).
setpoint_goal(0, 0, 0).

/* Initial goals */
!getGazeboOffset.
!setRTLAtlitude(3.0).
!inRange.

/* Plans */
+!setMaxSpeed(S) : state(_,"True",_)
  <-  set_fcu_param("MPC_XY_VEL_MAX", 0, S).

+!setMaxSpeed(S) <- !setMaxSpeed(S).

+!setRTLAtlitude(A) : state(_,"True",_)
  <-  set_fcu_param("RTL_RETURN_ALT", 0, A);
      set_fcu_param("RTL_DESCEND_ALT", 0, A-0.5).

+!setRTLAtlitude(A) <- !setRTLAtlitude(A).

+!inRange
	<-	.time(HH,MM,SS,MS);
			+range[ap(_),lu(HH,MM,SS,MS)].

+victim_in_need(N, GX, GY)[lu(HH,MM,SS,MS)]
	<-	+victim_position(N, GX, GY)[ap(100),lu(HH,MM,SS,MS)];
      !start_negotiation.

+!start_negotiation: .desire(negotiate) | .desire(rescueVictim)
	<- 	.suspend;
			!!start_negotiation.

@lg[atomic]
+!start_negotiation
	<- !negotiate.

+!negotiate
	<-	.findall([N, GX, GY], victim_in_need(N, GX, GY)[lu(HH,MM,SS,MS)], V);
			.sort(V, SV);
			.nth(0, SV, Next);
			Next = [N, GX, GY];

			!propose(N);
			.findall([O, Me], propose(Me, N, O), L);
			!choose_proposal(N, L).

-!negotiate.

+!propose(N)
	<- 	.random(R);
      .my_name(Me);
			+propose(Me, N, R*100);
			!broadcastProposal(10, Me, N, R*100).

+!broadcastProposal(I, Me, N, R) : I>0
  <-  !broadcast(tell, propose(Me, N, R));
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
			?victim_in_need(N, GX, GY);
      .abolish(victim_in_need(N, _, _));
      !broadcast(achieve, mark_as_rescued(N, Lat, Long));
      +victim_in_rescue(N, GX, GY);
      !!publishSetPoint;
			!!rescueVictim.


+!check_winner(N, Who)
	<- 	.abolish(victim_in_need(N, _, _));
      .print("Not selected to rescue victim ", N).

+!rescueVictim
	<- 	.wait(1000);
      ?victim_in_rescue(N, GX, GY);
      !defineGoalLocal([[GX, GY,_]]);
      !drop_buoy(N);
      -victim_in_rescue(N, _, _);
			!resume_negotiation.

+!publishSetPoint : state("OFFBOARD",_,"True")
	<-	?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(200);
			!publishSetPoint.

+!publishSetPoint
	<-	arm_motors(True);
			set_mode("OFFBOARD");
			?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(200);
			!publishSetPoint.

+!defineGoalLocal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
      .wait(gazebo_offset(_, _, _));
      ?gazebo_offset(OX, OY, OZ);
			-+setpoint_goal(X - OX, Y - OY, Z - OZ);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2+OX -(X)) <=0.7 & math.abs(Y2 +OY -(Y)) <=0.7 & math.abs(Z2 + OZ -(Z)) <=0.7);
			!defineGoalLocal(T).

+!defineGoalLocal([]).

+!drop_buoy(N)
  <-  ?flight_altitude(Z);
      !getGazeboPos;
      ?gazebo_pos(GX, GY, GZ);
      drop_buoy(GX, GY, GZ-0.25);
      .print("Droping buoy to victim ", N);
      .drop_intention(publishSetPoint);
      -+setpoint_goal(0,0,0);
      !broadcast(tell, victim_rescued(N));
      !returnHome.

+!resume_negotiation: .intend(start_negotiation)
  <-  .drop_intention(publishSetPoint);
      -+setpoint_goal(0,0,0);
      .resume(start_negotiation).

+!resume_negotiation <- .drop_intention(publishSetPoint); -+setpoint_goal(0,0,0);.

+!returnHome
  <-  .wait(local_pos(X,Y,Z,_,_,_,_) & X <=0.5 & Y <=0.5 & Z <= 0.2);
      .print("Landed! beginning charging and buoy replacement!");
      .wait(2000);
      .print("Completed recharging!!!!!!!!!!!!!").

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
      }.

+!getGazeboOffset
  <-  !getGazeboPos;
      ?gazebo_pos(GX, GY, GZ);
      ?local_pos(LX, LY, LZ, _, _, _, _);
      +gazebo_offset(GX-LX, GY-LY, GZ-LZ).

-!getGazeboOffset <- !getGazeboOffset.

+!mark_as_rescued(N, Lat, Long)
	<- .abolish(victim_in_need(N,Lat,Long)).

+!broadcast(Itl, Data) : range[ap(5000)]
	<- .broadcast(Itl, Data).

+!broadcast(Itl, Data).

+?range[ap]
  <-  .suspend(publishSetPoint);
      !flyToCommSpot(0, 6);
      .time(HH,MM,SS,MS);
  		+range[ap(_),lu(HH,MM,SS,MS)];
      .resume(publishSetPoint).


+!flyToCommSpot(X,Y): local_pos(LX, LY, _, _, _, _, _) & math.abs(X-LX) <=0.7 & math.abs(Y-LY) <=0.7
  <- .print("Arrived").

+!flyToCommSpot(X,Y): .intend(returnHome)
  <-  .print("Waiting to finish recharging");
      .wait(200);
      !flyToCommSpot(X,Y).

+!flyToCommSpot(X,Y): state("OFFBOARD",_,"True") & not .intend(returnHome)
  <-  ?flight_altitude(Z);
      setpoint_local(X,Y,Z);
      .wait(200);
      !flyToCommSpot(X,Y).

+!flyToCommSpot(X,Y): not state("OFFBOARD",_,"True") & not .intend(returnHome)
  <-  arm_motors(True);
			set_mode("OFFBOARD");
      ?flight_altitude(Z);
      setpoint_local(X,Y,Z);
      .wait(200);
      !flyToCommSpot(X,Y).

{apply_ap}
