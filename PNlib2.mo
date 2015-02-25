package PNlib2
  package Interfaces "contains the connectors for the Petri net component models"
    connector PlaceOut "part of place model to connect places to output transitions"
      input Boolean active "Are the output transitions active?" annotation(HideResult = true);
      input Boolean fire "Do the output transitions fire?" annotation(HideResult = true);
      input Real arcWeight "Arc weights of output transitions" annotation(HideResult = true);
      output Real t "Marking of the place" annotation(HideResult = true);
      output Real minTokens "Minimum capacity of the place" annotation(HideResult = true);
      output Boolean enable "Which of the output transitions are enabled by the place?" annotation(HideResult = true);
      output Boolean tokenInOut "Does the place have a discrete token change?" annotation(HideResult = true);
      annotation(Icon(graphics = {Polygon(points = {{-100, 100}, {98, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}));
    end PlaceOut;

    connector TransitionIn "part of transition model to connect transitions to input places"
      input Real t "Markings of input places" annotation(HideResult = true);
      input Real minTokens "Minimum capacites of input places" annotation(HideResult = true);
      input Boolean enable "Is the transition enabled by input places?" annotation(HideResult = true);
      input Boolean tokenInOut "Do the input places have a discrete token change?" annotation(HideResult = true);
      output Boolean active "Is the transition active?" annotation(HideResult = true);
      output Boolean fire "Does the transition fire?" annotation(HideResult = true);
      output Real arcWeight "Input arc weights of the transition" annotation(HideResult = true);
      annotation(Icon(graphics = {Polygon(points = {{-100, 100}, {98, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid)}));
    end TransitionIn;

    connector TransitionOut "part of transition model to connect transitions to output places"
      input Real t "Markings of output places" annotation(HideResult = true);
      input Real maxTokens "Maximum capacities of output places" annotation(HideResult = true);
      input Boolean enable "Is the transition enabled by output places?" annotation(HideResult = true);
      output Boolean active "Is the transition active?" annotation(HideResult = true);
      output Boolean fire "Does the transition fire?" annotation(HideResult = true);
      output Real arcWeight "Output arc weights of the transition" annotation(HideResult = true);
      output Boolean enabledByInPlaces "Is the transition enabled by all input places?" annotation(HideResult = true);
      annotation(Icon(graphics = {Polygon(points = {{-100, 100}, {98, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}));
    end TransitionOut;

    connector PlaceIn "part of place model to connect places to input transitions"
      input Boolean active "Are the input transitions active?" annotation(HideResult = true);
      input Boolean fire "Do the input transitions fire?" annotation(HideResult = true);
      input Real arcWeight "Arc weights of input transitions" annotation(HideResult = true);
      input Boolean enabledByInPlaces "Are the input transitions enabled by all theier input places?" annotation(HideResult = true);
      output Real t "Marking of the place" annotation(HideResult = true);
      output Real maxTokens "Maximum capacity of the place" annotation(HideResult = true);
      output Boolean enable "Which of the input transitions are enabled by the place?" annotation(HideResult = true);
      annotation(Icon(graphics = {Polygon(points = {{-100, 100}, {98, 0}, {-100, -100}, {-100, 100}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid)}));
    end PlaceIn;
  end Interfaces;

  package DiskretComponents
    model PD "Discrete Place "
      discrete Real t(start = startTokens) "marking";
      parameter Integer nIn = 0 "number of input transitions" annotation(Dialog(connectorSizing = true));
      parameter Integer nOut = 0 "number of output transitions" annotation(Dialog(connectorSizing = true));
      //****MODIFIABLE PARAMETERS AND VARIABLES BEGIN****//
      parameter Real startTokens = 0 "start tokens" annotation(Dialog(enable = true, group = "Tokens"));
      parameter Real minTokens = 0 "minimum capacity" annotation(Dialog(enable = true, group = "Tokens"));
      parameter Real maxTokens = PNlib2.Constants.inf "maximum capacity" annotation(Dialog(enable = true, group = "Tokens"));
      Boolean reStart = false "restart condition" annotation(Dialog(enable = true, group = "Tokens"));
      parameter Real reStartTokens = startTokens "number of tokens at restart" annotation(Dialog(enable = true, group = "Tokens"));
      parameter Integer N = settings1.N "N+1=amount of levels" annotation(Dialog(enable = true, group = "Level Concentrations"));
      //****MODIFIABLE PARAMETERS AND VARIABLES END****//
      Real levelCon "conversion of tokens to level concentration according to M and N of the settings box";
      Integer showPlaceName = settings1.showPlaceName "only for place animation and display (Do not change!)";
      Integer showCapacity = settings1.showCapacity "only for place animation and display (Do not change!)";
      Integer animateMarking = settings1.animateMarking "only for place animation and display (Do not change!)";
      Real color[3] "only for place animation and display (Do not change!)";
      //protected
      outer PNlib2.Settings settings1 "global settings for animation and display";
      Real tokenscale "only for place animation and display";
      //discrete Real pret "pre marking";
      Real pret "pre marking";
      Real arcWeightIn[nIn] "Integer weights of input arcs";
      Real arcWeightOut[nOut] "Integer weights of output arcs";
      Boolean tokeninout "change of tokens?";
      Boolean fireIn[nIn] "Do input transtions fire?";
      Boolean fireOut[nOut] "Do output transitions fire?";
      Boolean activeIn[nIn] "Are delays passed of input transitions?";
      Boolean activeOut[nOut] "Are delay passed of output transitions?";
      Boolean enabledByInPlaces[nIn] "Are input transitions are enabled by all their input places?";
      //****BLOCKS BEGIN****// since no events are gePNlib2.DiskretComponents.PDnerated within functions!!!
      //change of activation of output transitions
      PNlib2.Blocks.anyChange activeConOut(vec = pre(activeOut));
      //Does any delay passed of a connected transition?
      PNlib2.Blocks.anyTrue delayPassedOut(vec = activeOut);
      PNlib2.Blocks.anyTrue delayPassedIn(vec = activeIn);
      //firing sum calculation
      PNlib2.Blocks.firingSumDis firingSumIn(fire = fireIn, arcWeight = arcWeightIn);
      PNlib2.Blocks.firingSumDis firingSumOut(fire = fireOut, arcWeight = arcWeightOut);
      //Enabling process
      PNlib2.Blocks.enablingOutDis enableOut(delayPassed = delayPassedOut.anytrue, activeCon = activeConOut.anychange, nOut = nOut, arcWeight = arcWeightOut, t = pret, minTokens = minTokens, TAout = activeOut);
      PNlib2.Blocks.enablingInDis enableIn(delayPassed = delayPassedIn.anytrue, active = activeIn, nIn = nIn, arcWeight = arcWeightIn, t = pret, maxTokens = maxTokens, TAein = enabledByInPlaces);
      //****BLOCKS END****//
    public
      PNlib2.Interfaces.PlaceIn inTransition[nIn](each t = pret, each maxTokens = maxTokens, enable = enableIn.TEin_, fire = fireIn, arcWeight = arcWeightIn, active = activeIn, enabledByInPlaces = enabledByInPlaces) "connector for input transitions" annotation(Placement(transformation(extent = {{-114, -10}, {-98, 10}}, rotation = 0), iconTransformation(extent = {{-116, -10}, {-100, 10}})));
      PNlib2.Interfaces.PlaceOut outTransition[nOut](each t = pret, each minTokens = minTokens, enable = enableOut.TEout_, each tokenInOut = pre(tokeninout), fire = fireOut, arcWeight = arcWeightOut, active = activeOut) "connector for output transitions" annotation(Placement(transformation(extent = {{100, -10}, {116, 10}}, rotation = 0)));
      /*Modelica.Blocks.Interfaces.IntegerOutput pd_t=t 
                                                                                                                                                    "connector for Simulink connection"                                               annotation (Placement(
                                                                                                                                                        transformation(extent={{-36,68},{-16,88}}), iconTransformation(
                                                                                                                                                        extent={{-10,-10},{10,10}},
                                                                                                                                                        rotation=90,
                                                                                                                                                        origin={0,106})));*/
    equation
      //****MAIN BEGIN****//
      //recalculation of tokens
      pret = pre(t);
      tokeninout = firingSumIn.firingSum > 0 or firingSumOut.firingSum > 0;
      when tokeninout or pre(reStart) then
        t = if tokeninout then pret + firingSumIn.firingSum - firingSumOut.firingSum else reStartTokens;
      end when;
      //Conversion of tokens to level concentrations
      levelCon = t * settings1.M / N;
      //****MAIN END****//
      //****ANIMATION BEGIN****//
      tokenscale = t * settings1.scale;
      color = if settings1.animatePlace == 1 then if tokenscale < 100 then {255, 255 - 2.55 * tokenscale, 255 - 2.55 * tokenscale} else {255, 0, 0} else {255, 255, 255};
      //****ANIMATION END****//
      //****ERROR MESSENGES BEGIN****//
      assert(startTokens >= minTokens and startTokens <= maxTokens, "minTokens<=startTokens<=maxTokens");
      //****ERROR MESSENGES END****//
      annotation(defaultComponentName = "P1", Icon(graphics = {Ellipse(extent = {{-100, 96}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = DynamicSelect({255, 255, 255}, color), fillPattern = FillPattern.Solid), Text(extent = {{-1.5, 25.5}, {-1.5, -21.5}}, lineColor = {0, 0, 0}, textString = DynamicSelect("%startTokens", if animateMarking == 1 then realString(t, 1, 0) else " ")), Text(extent = {{-90, 130}, {-90, 116}}, lineColor = {0, 0, 0}, textString = DynamicSelect(" ", if showCapacity == 1 then if maxTokens > 1073741822 then "[%minTokens, inf]" else "[%minTokens, %maxTokens]" else " ")), Text(extent = {{-74, -113}, {-74, -138}}, lineColor = {0, 0, 0}, textString = "%name")}), Diagram(graphics));
    end PD;

    model TD "Discrete Transition"
      parameter Integer nIn = 0 "number of input places" annotation(Dialog(connectorSizing = true));
      parameter Integer nOut = 0 "number of output places" annotation(Dialog(connectorSizing = true));
      //****MODIFIABLE PARAMETERS AND VARIABLES BEGIN****//
      parameter Real delay = 1 "delay of timed transition" annotation(Dialog(enable = true, group = "Delay"));
      Real arcWeightIn[nIn] = fill(1, nIn) "arc weights of input places" annotation(Dialog(enable = true, group = "Arc Weights"));
      Real arcWeightOut[nOut] = fill(1, nOut) "arc weights of output places" annotation(Dialog(enable = true, group = "Arc Weights"));
      Boolean firingCon = true "additional firing condition" annotation(Dialog(enable = true, group = "Firing Condition"));
      //****MODIFIABLE PARAMETERS AND VARIABLES END****//
      Integer showTransitionName = settings1.showTransitionName "only for transition animation and display (Do not change!)";
      Integer showDelay = settings1.showDelay "only for transition animation and display (Do not change!)";
      Real color[3] "only for transition animation and display (Do not change!)";
      Boolean active "Is the transition active?";
      //protected
      outer PNlib2.Settings settings1 "global settings for animation and display";
      Real tIn[nIn] "tokens of input places";
      Real tOut[nOut] "tokens of output places";
      Real firingTime "next putative firing time";
      Real fireTime "for transition animation";
      Real minTokens[nIn] "minimum tokens of input places";
      Real maxTokens[nOut] "maximum tokens of output places";
      Real delay_ "due to problems if d==0";
      Boolean fire "Does the transition fire?";
      Boolean enableIn[nIn] "Is the transition enabled by input places?";
      Boolean enableOut[nOut] "Is the transition enabled by output places?";
      Boolean delayPassed "Is the delay passed?";
      Boolean ani "for transition animation";
      //****BLOCKS BEGIN****// since no events are generated within functions!!!
      //activation process
      PNlib2.Blocks.activationDis activation(nIn = nIn, nOut = nOut, tIn = tIn, tOut = tOut, arcWeightIn = arcWeightIn, arcWeightOut = arcWeightOut, minTokens = minTokens, maxTokens = maxTokens, firingCon = firingCon);
      //Is the transition enabled by all input places?
      PNlib2.Blocks.allTrue enabledByInPlaces(vec = enableIn);
      //Is the transition enabled by all output places?
      PNlib2.Blocks.allTrue enabledByOutPlaces(vec = enableOut);
      //****BLOCKS END****//
    public
      PNlib2.Interfaces.TransitionIn inPlaces[nIn](each active = delayPassed, arcWeight = arcWeightIn, each fire = fire, t = tIn, minTokens = minTokens, enable = enableIn) "connector for input places" annotation(Placement(transformation(extent = {{-56, -10}, {-40, 10}}, rotation = 0)));
      PNlib2.Interfaces.TransitionOut outPlaces[nOut](each active = delayPassed, arcWeight = arcWeightOut, each fire = fire, each enabledByInPlaces = enabledByInPlaces.alltrue, t = tOut, maxTokens = maxTokens, enable = enableOut) "connector for output places" annotation(Placement(transformation(extent = {{40, -10}, {56, 10}}, rotation = 0)));
    equation
      //****MAIN BEGIN****//
      delay_ = if delay <= 0 then 10 ^ (-6) else delay;
      //due to event problems if delay==0
      //reset active when delay passed
      active = activation.active and not pre(delayPassed);
      //save next putative firing time
      when active then
        firingTime = time + delay_;
      end when;
      //delay passed?
      delayPassed = active and time >= firingTime;
      //firing process
      fire = if nOut == 0 then enabledByInPlaces.alltrue else enabledByOutPlaces.alltrue;
      //****MAIN END****//
      //****ANIMATION BEGIN****//
      when fire then
        fireTime = time;
        ani = true;
      end when;
      color = if fireTime + settings1.timeFire >= time and settings1.animateTransition == 1 and ani then {255, 255, 0} else {0, 0, 0};
      //****ANIMATION END****//
      //****ERROR MESSENGES BEGIN****//
      for i in 1:nIn loop
        assert(arcWeightIn[i] >= 0, "Input arc weights must be positive.");
      end for;
      for i in 1:nOut loop
        assert(arcWeightOut[i] >= 0, "Output arc weights must be positive.");
      end for;
      //****ERROR MESSENGES END****//
      annotation(defaultComponentName = "T1", Icon(graphics = {Rectangle(extent = {{-40, 100}, {40, -100}}, lineColor = {0, 0, 0}, fillColor = DynamicSelect({0, 0, 0}, color), fillPattern = FillPattern.Solid), Text(extent = {{-2, -116}, {-2, -144}}, lineColor = {0, 0, 0}, textString = DynamicSelect("d=%delay", if showDelay == 1 then "d=%delay" else " ")), Text(extent = {{-4, 139}, {-4, 114}}, lineColor = {0, 0, 0}, textString = "%name")}), Diagram(graphics));
    end TD;
  end DiskretComponents;

  package Constants "contains constants which are used in the Petri net component models"
    constant Real inf = 1e+60 "Biggest Real number such that inf and -inf are representable on the machine";
    constant Integer Integer_inf = 1073741823 "Biggest Integer number such that Integer_inf and -Integer_inf are representable on the machine";
    constant Real pi = 2 * arcsin(1.0) "3.14159265358979";
    constant Real eps = 1.e-9 "Biggest number such that 1.0 + eps = 1.0";
    constant Real small = 1.e-60 "Smallest number such that small and -small are representable on the machine";
  end Constants;

  package Blocks "
                                                contains blocks with specific procedures that are used in the Petri net component models"
    block activationDis "Activation of a discrete transition"
      parameter input Integer nIn "number of input places";
      parameter input Integer nOut "number of output places";
      input Real tIn[:] "tokens of input places";
      input Real tOut[:] "tokens of output places";
      input Real arcWeightIn[:] "arc weights of input places";
      input Real arcWeightOut[:] "arc weights of output places";
      input Real minTokens[:] "minimum capacities of input places";
      input Real maxTokens[:] "maximum capacities of output places";
      input Boolean firingCon "firing condition of transition";
      output Boolean active "activation of transition";
    algorithm
      active := true;
      //check input places
      for i in 1:nIn loop
        if not tIn[i] + Constants.eps - arcWeightIn[i] >= minTokens[i] then
          active := false;
        end if;
      end for;
      //check output places
      for i in 1:nOut loop
        if not tOut[i] - Constants.eps + arcWeightOut[i] <= maxTokens[i] then
          active := false;
        end if;
      end for;
      active := active and firingCon;
    end activationDis;

    block allTrue "Are all entries of a Boolean vector true?"
      input Boolean vec[:];
      output Boolean alltrue;
    algorithm
      alltrue := true;
      for i in 1:size(vec, 1) loop
        alltrue := alltrue and vec[i];
      end for;
    end allTrue;

    block anyTrue "Is any entry of a Boolean vector true?"
      input Boolean vec[:];
      output Boolean anytrue;
      output Integer numtrue;
    algorithm
      anytrue := false;
      numtrue := 0;
      for i in 1:size(vec, 1) loop
        anytrue := anytrue or vec[i];
        if vec[i] then
          numtrue := numtrue + 1;
        end if;
      end for;
    end anyTrue;

    block anyChange "Does any entry of a Boolean vector change its value?"
      input Boolean vec[:];
      output Boolean anychange;
    algorithm
      anychange := false;
      for i in 1:size(vec, 1) loop
        anychange := anychange or change(vec[i]);
      end for;
    end anyChange;

    block enablingInDis "enabling process of discrete input transitions"
      parameter input Integer nIn "number of input transitions";
      input Real arcWeight[:] "arc weights of input transitions";
      input Real t "current token number";
      input Real maxTokens "maximum capacity";
      input Boolean TAein[:] "active previous transitions which are already enabled by their input places";
      input Boolean delayPassed "Does any delayPassed of a output transition";
      input Boolean active[:] "Are the input transitions active?";
      output Boolean TEin_[nIn] "enabled input transitions";
    protected
      Boolean TEin[nIn] "enabled input transitions";
      Real arcWeightSum "arc weight sum";
    algorithm
      TEin := fill(false, nIn);
      when delayPassed then
        if nIn > 0 then
          arcWeightSum := PNlib2.Functions.OddsAndEnds.conditionalSum(arcWeight, TAein);
          if t + arcWeightSum <= maxTokens then
            TEin := TAein;
          end if;
        else
          arcWeightSum := 0;
        end if;
      end when;
      //arc weight sum of all active input transitions which are already enabled by their input places
      //Place has no actual conflict; all active input transitions are enabled
      TEin_ := TEin and active;
    end enablingInDis;

    block enablingOutDis "enabling process of output transitions"
      parameter input Integer nOut "number of output transitions";
      input Real arcWeight[:] "arc weights of output transitions";
      input Real t "current token number";
      input Real minTokens "minimum capacity";
      input Boolean TAout[:] "active output transitions with passed delay";
      input Boolean delayPassed "Does any delayPassed of a output transition";
      input Boolean activeCon "change of activation of output transitions";
      output Boolean TEout_[nOut] "enabled output transitions";
    protected
      Boolean TEout[nOut] "enabled input transitions";
      Real arcWeightSum "arc weight sum";
    algorithm
      TEout := fill(false, nOut);
      when delayPassed or activeCon then
        if nOut > 0 then
          arcWeightSum := PNlib2.Functions.OddsAndEnds.conditionalSum(arcWeight, TAout);
          if t - arcWeightSum >= minTokens then
            TEout := TAout;
          end if;
        else
          arcWeightSum := 0;
        end if;
      end when;
      //arc weight sum of all active output transitions
      //Place has no actual conflict; all active output transitions are enabled
      TEout_ := TEout and TAout;
    end enablingOutDis;

    block firingSumDis "calculates the firing sum of discrete places"
      input Boolean fire[:] "firability of transitions";
      input Real arcWeight[:] "arc weights";
      output Real firingSum "firing sum";
    algorithm
      firingSum := 0;
      for i in 1:size(fire, 1) loop
        if fire[i] then
          firingSum := firingSum + arcWeight[i];
        end if;
      end for;
    end firingSumDis;
  end Blocks;

  package Functions "contains functions with specific algorithmic procedures which are used in the Petri net component models"
    package Random "random functions"
      impure function initRandom "(re)initialize random number generator"
        impure function initRandom_help "external C-function to (re)initialize random number generator"
          input Integer seed;
        
          external "C" _initRandom(seed) annotation(Include = "
                                                                                                                                                                                                                                                                          #include <stdlib.h>
                                                                                                                                                                                                                                                                          int _initRandom(int seed)
                                                                                                                                                                                                                                                                          {
                                                                                                                                                                                                                                                                           srand(seed);
                                                                                                                                                                                                                                                                          }");
        end initRandom_help;

        input Integer seed;
      algorithm
        random();
        initRandom_help(seed);
      end initRandom;

      impure function random "external C-function to generate uniform distributed random numbers"
        output Integer x;
      
        external "C" x = _random() annotation(Include = "#include <stdlib.h>
                                                                                                                                                                                                                                                                           int _random()
                                                                                                                                                                                                                                                                           {
                                                                                                                                                                                                                                                                             static int called=0;
                                                                                                                                                                                                                                                                             int i;
                                                                                                                                                                                                                                                                             if(!called)
                                                                                                                                                                                                                                                                             {
                                                                                                                                                                                                                                                                               srand((unsigned) time(NULL));
                                                                                                                                                                                                                                                                               called=1;
                                                                                                                                                                                                                                                                             }
                                                                                                                                                                                                                                                                             return rand();
                                                                                                                                                                                                                                                                           }");
      end random;

      impure function randomexp "generates a exponential distributed random number according to lambda"
        input Real lambda;
        output Real delay;
      protected
        Real zg;
        Real h_lambda;
      algorithm
        zg := 0;
        h_lambda := lambda;
        while zg / 32767 < 10 ^ (-10) loop
          zg := random();
        end while;
        if lambda <= 0 then
          h_lambda := 10 ^ (-10);
        end if;
        delay := -Modelica.Math.log(zg / 32767) * 1 / h_lambda;
      end randomexp;
    end Random;

    package OddsAndEnds "help functions"
      function allTrue "Are all entries of a boolean vector true?"
        input Boolean vec[:];
        output Boolean alltrue;
      algorithm
        alltrue := true;
        for i in 1:size(vec, 1) loop
          alltrue := alltrue and vec[i];
        end for;
      end allTrue;

      function anyTrue "Is any entry of a boolean vector true"
        input Boolean vec[:];
        output Boolean anytrue;
      algorithm
        anytrue := false;
        for i in 1:size(vec, 1) loop
          anytrue := anytrue or vec[i];
        end for;
      end anyTrue;

      function boolToInt "converts a boolean to an integer"
        input Boolean bool;
        output Integer int;
      algorithm
        if bool then
          int := 1;
        else
          int := 0;
        end if;
      end boolToInt;

      function compare "compare two simulation results (test suite)"
        input Real time_org[:];
        input Real time_test[:];
        input Real var_org[:];
        input Real var_test[:];
        input Real interval;
        input Real stopTime;
        output Real err;
      protected
        Integer j;
        Integer k;
      algorithm
        j := 1;
        k := 1;
        err := 0;
        for i in 0:interval:stopTime loop
          while time_org[j] <= i loop
            if j < size(time_org, 1) then
              j := j + 1;
            else
              break;
            end if;
          end while;
          while time_test[k] <= i loop
            if k < size(time_test, 1) then
              k := k + 1;
            else
              break;
            end if;
          end while;
          err := err + (var_org[j - 1] - var_test[k - 1]) ^ 2;
        end for;
      end compare;

      function compareReal "compare two real vectors"
        input Real vec1[:];
        input Real vec2[:];
        output Boolean vec3[size(vec1, 1)];
      algorithm
        for i in 1:size(vec1, 1) loop
          vec3[i] := vec1[i] > vec2[i];
        end for;
      end compareReal;

      function conditionalSum "calculates the conditional sum of real vector entries"
        input Real vec[:];
        input Boolean con[:];
        output Real conSum;
      algorithm
        conSum := 0;
        for i in 1:size(vec, 1) loop
          if con[i] then
            conSum := conSum + vec[i];
          end if;
        end for;
      end conditionalSum;

      function countTrue "counts the true entries of a Boolean vectors"
        input Boolean vec[:];
        output Integer count;
        output Integer idx[size(vec, 1)];
      algorithm
        count := 0;
        idx := zeros(size(vec, 1));
        for i in 1:size(vec, 1) loop
          if vec[i] then
            count := count + 1;
            idx[count] := i;
          end if;
        end for;
      end countTrue;

      function deleteElement "deletes an element of an real vector"
        input Real vecin[:];
        input Integer idx;
        output Real vecout[size(vecin, 1)];
      protected
        parameter Integer nVec = size(vecin, 1);
      algorithm
        vecout[1:idx - 1] := vecin[1:idx - 1];
        vecout[idx:nVec - 1] := vecin[idx + 1:nVec];
        vecout[nVec] := 0;
      end deleteElement;

      function deleteElementInt "deletes an element of an integer vector"
        input Integer vecin[:];
        input Integer idx;
        output Integer vecout[size(vecin, 1)];
      protected
        parameter Integer nVec = size(vecin, 1);
      algorithm
        vecout[1:idx - 1] := vecin[1:idx - 1];
        vecout[idx:nVec - 1] := vecin[idx + 1:nVec];
        vecout[nVec] := 0;
      end deleteElementInt;

      function findFirstZero "finds the first zero entry of an integer vector and returns its index; returns 0 if the vector has no zero entry"
        input Integer vec[:];
        output Integer idx;
      protected
        parameter Integer nSize = size(vec, 1);
        Boolean endAlg;
        Integer k;
      algorithm
        endAlg := false;
        k := 1;
        idx := 0;
        while not endAlg and k <= nSize loop
          if vec[k] == 0 then
            idx := k;
            endAlg := true;
          else
            k := k + 1;
          end if;
        end while;
      end findFirstZero;

      function isEqual "Determine if two Real scalars are numerically identical"
        extends Modelica.Icons.Function;
        input Real s1 "First scalar";
        input Real s2 "Second scalar";
        input Real eps(min = 0) = 0 "The two scalars are identical if abs(s1-s2) <= eps";
        output Boolean result "= true, if scalars are identical";
      algorithm
        result := abs(s1 - s2) <= eps;
        annotation(Inline = true, Documentation(info = "<HTML>
<h4>Syntax</h4>
<blockquote><pre>
Math.<b>isEqual</b>(s1, s2);
Math.<b>isEqual</b>(s1, s2, eps=0);
</pre></blockquote>
<h4>Description</h4>
<p>
The function call \"<code>Math.isEqual(s1, s2)</code>\" returns <b>true</b>,
if the two Real scalars s1 and s2 are identical. Otherwise the function
returns <b>false</b>. The equality check is performed by
\"abs(s1-s2) &le; eps\", where \"eps\"
can be provided as third argument of the function. Default is \"eps = 0\".
</p>
<h4>Example</h4>
<blockquote><pre>
  Real s1 = 2.0;
  Real s2 = 2.0;
  Real s3 = 2.000001;
  Boolean result;
<b>algorithm</b>
  result := Math.isEqual(s1,s2);     // = <b>true</b>
  result := Math.isEqual(s1,s3);     // = <b>false</b>
  result := Math.isEqual(s1,s3,0.1); // = <b>true</b>
</pre></blockquote>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Math.Vectors.isEqual\">Vectors.isEqual</a>,
<a href=\"modelica://Modelica.Math.Matrices.isEqual\">Matrices.isEqual</a>,
<a href=\"modelica://Modelica.Utilities.Strings.isEqual\">Strings.isEqual</a>
</p>
</HTML>"));
      end isEqual;

      function printResult "print the test result (test suite)"
        input Boolean ok;
        input String name;
      algorithm
        if ok then
          Modelica.Utilities.Streams.print(name + ": ok");
        else
          Modelica.Utilities.Streams.print(name + ": not ok!!!");
        end if;
      end printResult;

      function sort "Sort elements of vector in ascending or descending order"
        extends Modelica.Icons.Function;
        input Real v[:] "Vector to be sorted";
        input Boolean ascending = true "= true if ascending order, otherwise descending order";
        output Real sorted_v[size(v, 1)] = v "Sorted vector";
        output Integer indices[size(v, 1)] = 1:size(v, 1) "sorted_v = v[indices]";
        /* shellsort algorithm; should be improved later */
      protected
        Integer gap;
        Integer i;
        Integer j;
        Real wv;
        Integer wi;
        Integer nv = size(v, 1);
        Boolean swap;
      algorithm
        gap := div(nv, 2);
        while gap > 0 loop
          i := gap;
          while i < nv loop
            j := i - gap;
            if j >= 0 then
              if ascending then
                swap := sorted_v[j + 1] > sorted_v[j + gap + 1];
              else
                swap := sorted_v[j + 1] < sorted_v[j + gap + 1];
              end if;
            else
              swap := false;
            end if;
            while swap loop
              wv := sorted_v[j + 1];
              wi := indices[j + 1];
              sorted_v[j + 1] := sorted_v[j + gap + 1];
              sorted_v[j + gap + 1] := wv;
              indices[j + 1] := indices[j + gap + 1];
              indices[j + gap + 1] := wi;
              j := j - gap;
              if j >= 0 then
                if ascending then
                  swap := sorted_v[j + 1] > sorted_v[j + gap + 1];
                else
                  swap := sorted_v[j + 1] < sorted_v[j + gap + 1];
                end if;
              else
                swap := false;
              end if;
            end while;
            i := i + 1;
          end while;
          gap := div(gap, 2);
        end while;
        annotation(Documentation(info = "<HTML>
<h4>Syntax</h4>
<blockquote><pre>
           sorted_v = Vectors.<b>sort</b>(v);
(sorted_v, indices) = Vectors.<b>sort</b>(v, ascending=true);
</pre></blockquote>
<h4>Description</h4>
<p>
Function <b>sort</b>(..) sorts a Real vector v
in ascending order and returns the result in sorted_v.
If the optional argument \"ascending\" is <b>false</b>, the vector
is sorted in descending order. In the optional second
output argument the indices of the sorted vector with respect
to the original vector are given, such that sorted_v = v[indices].
</p>
<h4>Example</h4>
<blockquote><pre>
  (v2, i2) := Vectors.sort({-1, 8, 3, 6, 2});
       -> v2 = {-1, 2, 3, 6, 8}
          i2 = {1, 5, 3, 4, 2}
</pre></blockquote>
</HTML>"));
      end sort;

      function testOK "calculates if the test is ok if a Petri net has stochastic effects (test suite)"
        input Real err[:];
        input String name;
        input Real tol = 0.5;
        output Boolean ok;
      protected
        Integer i;
      algorithm
        ok := false;
        i := 1;
        while not ok and i <= size(err, 1) loop
          if err[i] < tol then
            ok := true;
          end if;
          i := i + 1;
        end while;
        printResult(ok, name);
      end testOK;
    end OddsAndEnds;
  end Functions;

  model Settings "Global Settings for Animation and Display"
    parameter Integer showPlaceName = 1 "show names of places" annotation(Dialog(enable = true, group = "Display"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer showTransitionName = 1 "show names of transitions" annotation(Dialog(enable = true, group = "Display"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer showDelay = 1 "show delays of discrete transitions" annotation(Dialog(enable = true, group = "Display"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer showCapacity = 2 "show capacities of places" annotation(Dialog(enable = true, group = "Display"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer animateMarking = 1 "animation of markings" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer animatePlace = 1 "animation of places" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Real scale = 1 "scale factor for place animation 0-100" annotation(Dialog(enable = if animationPlace == 1 then true else false, group = "Animation"));
    parameter Integer animateTransition = 1 "animation of transitions" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Real timeFire = 0.3 "time of transition animation" annotation(Dialog(enable = if animationTransition == 1 then true else false, group = "Animation"));
    parameter Integer animatePutFireTime = 1 "animation of putative fire time of stochastic transitions" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer animateHazardFunc = 1 "animation of hazard functions of stochastic transitions" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer animateSpeed = 1 "animation speed of continuous transitions" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer animateWeightTIarc = 1 "show weights of test and inhibitor arcs" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer animateTIarc = 1 "animation of test and inhibition arcs" annotation(Dialog(enable = true, group = "Animation"), choices(choice = 1 "on", choice = 2 "off", __Dymola_radioButtons = true));
    parameter Integer N = 10 "N+1=amount of levels" annotation(Dialog(enable = true, group = "Level Concentrations"));
    parameter Real M = 1 "maximum concentration" annotation(Dialog(enable = true, group = "Level Concentrations"));
    annotation(defaultComponentName = "settings1", defaultComponentPrefixes = "inner", missingInnerMessage = "The settings object is missing", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-98, 98}, {98, -98}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.Solid, fillColor = {255, 255, 255}), Text(extent = {{0, 22}, {0, -22}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, textString = "SETTINGS")}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics));
  end Settings;

  package Examples
    model Test1
      DiskretComponents.TD T1(arcWeightOut = {1 / 3}, nOut = 1) annotation(Placement(transformation(extent = {{-84, -2}, {-64, 18}})));
      DiskretComponents.PD P2(startTokens = 0, nIn = 1, nOut = 1) annotation(Placement(transformation(extent = {{-8, -2}, {12, 18}})));
      DiskretComponents.TD T2(delay = 1, nIn = 1, nOut = 1, arcWeightIn = {1 / 3}, arcWeightOut = {1 / 3}) annotation(Placement(transformation(extent = {{-32, -2}, {-12, 18}})));
      DiskretComponents.PD P3(startTokens = 0, nIn = 1, nOut = 1) annotation(Placement(transformation(extent = {{46, -2}, {66, 18}})));
      DiskretComponents.TD T3(nIn = 1, nOut = 1, arcWeightIn = {1 / 3}, arcWeightOut = {1 / 3}) annotation(Placement(transformation(extent = {{16, -2}, {36, 18}})));
      DiskretComponents.PD P1(nIn = 1, nOut = 1) annotation(Placement(transformation(extent = {{-58, -2}, {-38, 18}})));
      inner PNlib2.Settings settings1 annotation(Placement(visible = true, transformation(origin = {44, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DiskretComponents.TD T4(nIn = 1, nOut = 1, arcWeightIn = {1 / 3}, arcWeightOut = {1 / 3}) annotation(Placement(transformation(extent = {{66, -2}, {86, 18}})));
      DiskretComponents.PD P4(nIn = 1) annotation(Placement(transformation(extent = {{88, -2}, {108, 18}})));
    equation
      connect(T1.outPlaces[1], P1.inTransition[1]) annotation(Line(points = {{-69.2, 8}, {-58.8, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      connect(P1.outTransition[1], T2.inPlaces[1]) annotation(Line(points = {{-37.2, 8}, {-26.8, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      connect(T2.outPlaces[1], P2.inTransition[1]) annotation(Line(points = {{-17.2, 8}, {-8.8, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      connect(P2.outTransition[1], T3.inPlaces[1]) annotation(Line(points = {{12.8, 8}, {21.2, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      connect(T3.outPlaces[1], P3.inTransition[1]) annotation(Line(points = {{30.8, 8}, {45.2, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      connect(P3.outTransition[1], T4.inPlaces[1]) annotation(Line(points = {{66.8, 8}, {71.2, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      connect(T4.outPlaces[1], P4.inTransition[1]) annotation(Line(points = {{80.8, 8}, {87.2, 8}}, color = {0, 0, 0}, smooth = Smooth.None));
      annotation(Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics));
    end Test1;

    model Test2
      DiskretComponents.PD P1 annotation(Placement(transformation(extent = {{-8, -10}, {12, 10}})));
      inner Settings settings1 annotation(Placement(transformation(extent = {{42, 48}, {62, 68}})));
    end Test2;
  end Examples;
  annotation(uses(Modelica(version = "3.2.1"), PNlib(version = "1.1")));
end PNlib2;