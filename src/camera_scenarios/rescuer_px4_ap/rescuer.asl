mode("land").
// flight_altitude(1.5).
scout_offset(1.5).

!setFlightAltitude.
!setRTLAtlitude(3).
!startPublishingSetPoints.

// !cameraOff.
//
// +!cameraOff <- camera_switch(False).

+!setFlightAltitude
  <-  .random(R);
      +flight_altitude(1.75+R*0.5).

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
      camera_switch(False);
      -+mode("Fly");
      // .abolish(victim(_,_)[ap(_)]);
      !setMaxSpeed(12);
      !defineGoal([[Lat, Long,_]]);
      !drop_buoy(N);
      .broadcast(tell, victim_rescued(N, Lat, Long));
			!resume_negotiation.

// +!drop_buoy(N) : victim(VX, VY)[ap(500)]
+!drop_buoy(N) : victim[ap(500)]
  <-  ?flight_altitude(Z);
      // -+setpoint_goal(VX,VY,Z);
      // .wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(VX)) <=0.5 & math.abs(Y2 -(VY)) <=0.5);
      !getGazeboPos;
      ?gazebo_pos(GX, GY, GZ);
      drop_buoy(GX, GY, GZ-0.25);
      .print("Droping buoy to victim ", N);
      !returnHome.

+!drop_buoy(N)
  <-  .print("Victim ", N, " not found").

// +?victim(VX, VY)[ap(T)]
+?victim[ap(T)]
  <-  .print("Active perception for victim!!!!!");
      camera_switch(True);
      .wait(1000);
      ?local_pos(X, Y, Z, QX, QY, QZ, QW);
      ?scout_offset(SO);
      plan_path(X, Y, Z, QX, QY, QZ, QW, [[X-SO, Y-SO, 0],[X-SO, Y+SO, 0],[X+SO, Y+SO, 0],[X+SO, Y-SO, 0]]);
      .wait(plan_path_result(PLIST));
      -+mode("scout");
      !setMaxSpeed(2);
      !defineGoalLocal(PLIST);
      camera_switch(False);
      .

+!resume_negotiation: .intend(start_negotiation)
  <- .resume(start_negotiation).

+!resume_negotiation <- !returnHome.

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

+!defineGoalLocal([H|T])
	<- 	H = [X, Y, _];
			?flight_altitude(Z);
			-+setpoint_goal(X,Y,Z);
			.wait(local_pos(X2,Y2,Z2,_,_,_,_) & math.abs(X2 -(X)) <=0.5 & math.abs(Y2 -(Y)) <=0.5 & math.abs(Z2 -(Z)) <=0.5);
			!defineGoalLocal(T).

+!defineGoalLocal([]).

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

+!publishSetPoint : mode("scout")
	<-	?setpoint_goal(X,Y,Z);
			setpoint_local(X,Y,Z);
			.wait(100);
			!publishSetPoint.

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

+object_detection(OBJ_LIST)
	<- !getObjects(OBJ_LIST).
// +object_tracker(OBJ_LIST)
// 	<- !getObjects(OBJ_LIST).

+!getObjects([H|T])
	<-	H = [Label, ID, Score];
			.term2string(Label, LabelStr);
			if(LabelStr == "victim"){
        // ?local_pos(X,Y,_,_,_,_,_);
        .time(HH,MM,SS,MS);
        // .abolish(victim(_,_)[ap(_)]);
        // +victim(X,Y)[ap(3000),lu(HH,MM,SS,MS)];
        +victim[ap(500),lu(HH,MM,SS,MS)];
        // camera_switch(False);
        .succeed_goal(defineGoalLocal(_));
			}else{
        !getObjects(T);
      }.

+!getObjects([]).

{apply_ap}
