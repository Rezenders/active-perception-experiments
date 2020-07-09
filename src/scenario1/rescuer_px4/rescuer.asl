/* Initial beliefs and rules */
mode("land").
flight_altitude(2).

/* Initial goals */
!setRTLAtlitude(3).
!startPublishingSetPoints.

/* Plans */
+!setMaxSpeed(S)
  <- set_fcu_param("MPC_XY_VEL_MAX", 0, S).

+!setRTLAtlitude(A)
  <-  set_fcu_param("RTL_RETURN_ALT", 0, A);
      set_fcu_param("RTL_DESCEND_ALT", 0, A-0.5).

+!startPublishingSetPoints
  <-  .wait(home_pos(HLat, HLong, _));
      ?home_pos(HLat, HLong, _);
      -+setpoint_goal(HLat, HLong, 0);
      !!publishSetPoint.

+victim_in_need(N, Lat, Long)[lu(HH,MM,SS,MS)]
	<-	+victim_position(N, Lat, Long)[ap(100),lu(HH,MM,SS,MS)];
      !start_negotiation.

+victim(ID)
  <-  ?global_pos(Lat, Long, _);
      .time(HH,MM,SS,MS);
      ?victim_in_rescue(N, Lat2, Long2);
      .abolish(victim_position(N, _, _));
      .print("DETECTED VICTIM");
      +victim_position(N, Lat, Long)[ap(500),lu(HH,MM,SS,MS)].

+!start_negotiation: .desire(negotiate) | .desire(rescueVictim)
	<- 	.suspend;
			!!start_negotiation.

@lg[atomic]
+!start_negotiation
	<- !negotiate.

+!negotiate
	<-	.findall([N, Lat, Long], victim_in_need(N,Lat,Long)[lu(HH,MM,SS,MS)], V);
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
      .abolish(victim_in_need(N, _, _));
      +victim_in_rescue(N, Lat, Long);
			.broadcast(achieve, mark_as_rescued(N, Lat, Long));
      .resume(publishSetPoint);
			!!rescueVictim.

+!check_winner(N, Who)
	<- 	.print("Not selected!").

+!rescueVictim
	<- 	.wait(1000);
      -+mode("Fly");
      !setMaxSpeed(12);
      ?victim_in_rescue(N, Lat, Long);
      !defineGoal([[Lat, Long,_]]);
      !drop_buoy(N);
      -victim_in_rescue(N, _, _);
			!resume_negotiation.

+!drop_buoy(N): victim_position(N, Lat, Long)[ap(500)]
  <-  ?flight_altitude(Z);
      !getGazeboPos;
      ?gazebo_pos(GX, GY, GZ);
      drop_buoy(GX, GY, GZ-0.25);
      .print("Droping buoy to victim ", N);
      .broadcast(tell, victim_rescued(N));
      !returnHome.

+!drop_buoy(N)
  <-  .print("Victim ", N, " not found");
      .broadcast(tell, victim_drowned(N)).

+?victim_position(N, Lat, Long)[ap(T)]
  <-  .print("Active perception for victim ", N," !!!!!!!!!!!!!!!!!!!!!!!!!!!");
      camera_switch(True);
      .wait(2000);
      camera_switch(False);
      .

+!resume_negotiation: .intend(start_negotiation)
  <-  .resume(start_negotiation).

// +!resume_negotiation <- !returnHome.
+!resume_negotiation <- .suspend(publishSetPoint).

+!mark_as_rescued(N, Lat, Long)
	<- .abolish(victim_in_need(N,Lat,Long)).

+!defineGoal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
      ?altitude(A);
      ?global_pos(_,_,GZ);
      ?home_pos(_,_,HA);
			-+setpoint_goal(X, Y, HA - (GZ-A) + Z);
			.wait(global_pos(X2,Y2,Z2) & math.abs(X2 -(X)) <=0.00001 & math.abs(Y2 -(Y)) <=0.00001);
			!defineGoal(T).

+!defineGoal([]).

// +!defineGoalLocal([H|T])
// 	<- 	H = [X, Y, _];
// 			?flight_altitude(Z);
// 			-+setpoint_goal(X,Y,Z);
// 			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.5 & math.abs(Y2 -(Y)) <=0.5 & math.abs(Z2 -(Z)) <=0.5);
// 			!defineGoalLocal(T).
//
// +!defineGoalLocal([]).

+!publishSetPoint : (mode("Fly") & state("OFFBOARD",_,"True")) | (mode("land"))
	<-	?setpoint_goal(X,Y,Z);
			setpoint_global(X,Y,Z);
			.wait(100);
			!publishSetPoint.

+!publishSetPoint : mode("Fly") & not state("OFFBOARD",_,"True")
	<-	arm_motors(True);
			set_mode("OFFBOARD");
			?setpoint_goal(X,Y,Z);
			setpoint_global(X,Y,Z);
			.wait(100);
			!publishSetPoint.

// +!publishSetPoint : mode("scout")
// 	<-	?setpoint_goal(X,Y,Z);
// 			setpoint_local(X,Y,Z);
// 			.wait(100);
// 			!publishSetPoint.

+!publishSetPoint <- !publishSetPoint.

+!returnHome
  <-  .suspend(publishSetPoint);
      !setMaxSpeed(12);
      -+mode("land");
      ?home_pos(X2,Y2,Z2);
      -+setpoint_goal(X2, Y2, Z2);
      .wait(global_pos(X,Y,Z) & math.abs(X2 -(X)) <=0.00001 & math.abs(Y2 -(Y)) <=0.00001 & math.abs(Z2 -(Z)) <= 0.1);
      .print("Landed! beginning charging and buoy replacement!");
      .wait(1000).

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

+!setMode(Mode) : not state(Mode,_,_)
  <-  set_mode(Mode);
      !setMode(Mode).

+!setMode(Mode).

{apply_ap}
