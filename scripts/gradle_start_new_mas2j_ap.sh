#!/bin/bash
echo "MAS $2 {

    infrastructure: Centralised

    agents: $2 $3 agentArchClass jasonros.RosArch;
    directives: apply_ap = active_perception.Directive3;
}" > $1/$2.masj

gradle run -b $1/build.gradle -Pmas2j=$1/$2.masj
