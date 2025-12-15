package differential_drive
  package Sensors
    model AbsoluteOdometryImu
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput position[3](each final quantity = "Length", each final unit = "m") annotation(
        Placement(visible = true, transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput orientation[4] annotation(
        Placement(visible = true, transformation(origin = {-30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput linear_velocity[3](each final quantity = "Velocity", each final unit = "m/s") annotation(
        Placement(visible = true, transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput angular_velocity[3](each final quantity = "AngularVelocity", each final unit = "1/s") annotation(
        Placement(visible = true, transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput linear_acceleration[3](each final quantity = "Acceleration", each final unit = "m/s2") annotation(
        Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
      Sensors.AbsoluteOrientation absoluteOrientation annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor multisensor(animation = false, get_a = true, get_angles = false, get_r = true, get_v = true, get_w = true, get_z = true, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Derivative derivative[3] annotation(
        Placement(visible = true, transformation(origin = {-60, -30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(absoluteOrientation.frame_a, frame_a) annotation(
        Line(points = {{-80, 70}, {-90, 70}, {-90, 0}, {-102, 0}}, color = {95, 95, 95}));
      connect(multisensor.frame_a, frame_a) annotation(
        Line(points = {{-80, 30}, {-90, 30}, {-90, 0}, {-102, 0}}));
      connect(multisensor.r, position) annotation(
        Line(points = {{-80, 20}, {-80, -70}, {-30, -70}}, color = {0, 0, 127}, thickness = 0.5));
      connect(multisensor.v, linear_velocity) annotation(
        Line(points = {{-76, 20}, {-76, -50}, {-30, -50}}, color = {0, 0, 127}, thickness = 0.5));
      connect(multisensor.w, angular_velocity) annotation(
        Line(points = {{-64, 20}, {-64, -10}, {-30, -10}}, color = {0, 0, 127}, thickness = 0.5));
      connect(absoluteOrientation.quaternion, orientation) annotation(
        Line(points = {{-60, 70}, {-30, 70}}, color = {0, 0, 127}, thickness = 0.5));
  connect(derivative.y, linear_acceleration) annotation(
        Line(points = {{-54, -30}, {-30, -30}}, color = {0, 0, 127}, thickness = 0.5));
  connect(derivative.u, multisensor.v) annotation(
        Line(points = {{-68, -30}, {-76, -30}, {-76, 20}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Icon(graphics = {Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Ellipse(fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, extent = {{-70, -70}, {70, 70}}), Ellipse(fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-7, -7}, {7, 7}}), Line(points = {{-37.6, 13.7}, {-65.8, 23.9}}), Line(points = {{-22.9, 32.8}, {-40.2, 57.3}}), Text(textColor = {128, 128, 128}, extent = {{-108, 43}, {-72, 18}}, textString = "a"), Line(points = {{22.9, 32.8}, {40.2, 57.3}}), Line(points = {{0, 70}, {0, 40}}), Text(origin = {0, -6}, textColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Line(points = {{37.6, 13.7}, {65.8, 23.9}}), Ellipse(lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-12, -12}, {12, 12}}), Polygon(rotation = -17.5, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-5, 0}, {-2, 60}, {0, 65}, {2, 60}, {5, 0}, {-5, 0}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end AbsoluteOdometryImu;

    model AbsoluteOrientation
      Modelica.Blocks.Interfaces.RealOutput quaternion[4] annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
        Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    equation
      assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
      frame_a.f = zeros(3);
      frame_a.t = zeros(3);
      quaternion[4] = sqrt(1 + frame_a.R.T[1, 1] + frame_a.R.T[2, 2] + frame_a.R.T[3, 3])/2;
      quaternion[1] = (frame_a.R.T[3, 2] - frame_a.R.T[2, 3])/(4*quaternion[4]);
      quaternion[2] = (frame_a.R.T[1, 3] - frame_a.R.T[3, 1])/(4*quaternion[4]);
      quaternion[3] = (frame_a.R.T[2, 1] - frame_a.R.T[1, 2])/(4*quaternion[4]);
      annotation(
        Icon(graphics = {Text(origin = {0, -6}, textColor = {0, 0, 255}, extent = {{-132, 76}, {129, 124}}, textString = "%name"), Line(points = {{-70, 0}, {-96, 0}, {-96, 0}}), Ellipse(fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, extent = {{-70, -70}, {70, 70}}), Line(points = {{22.9, 32.8}, {40.2, 57.3}}), Line(points = {{37.6, 13.7}, {65.8, 23.9}}), Line(points = {{0, 70}, {0, 40}}), Ellipse(lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-12, -12}, {12, 12}}), Polygon(rotation = -17.5, fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-5, 0}, {-2, 60}, {0, 65}, {2, 60}, {5, 0}, {-5, 0}}), Line(points = {{-22.9, 32.8}, {-40.2, 57.3}}), Line(points = {{-37.6, 13.7}, {-65.8, 23.9}}), Ellipse(fillColor = {64, 64, 64}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-7, -7}, {7, 7}}), Text(textColor = {128, 128, 128}, extent = {{-108, 43}, {-72, 18}}, textString = "a")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end AbsoluteOrientation;
  end Sensors;

  package Simulators
    model kinematic_simulator
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
        Placement(visible = true, transformation(origin = {0, -10}, extent = {{-100, -10}, {-80, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 0.05, I_22 = 0.05, I_33 = 0.05, animation = false, m = 10, r_CM = {0, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {70, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_x(animation = false, n = {1, 0, 0}, useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_y(animation = false, n = {0, 1, 0}, useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {-10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Revolute revolute_yaw(animation = false, n = {0, 0, 1}, useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Speed speed_w(exact = true) annotation(
        Placement(visible = true, transformation(origin = {10, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Visualizers.FixedShape body_shape(height = 0.1, length = 0.3, width = 0.1) annotation(
        Placement(visible = true, transformation(origin = {72, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Translational.Sources.Speed speed_y(exact = true) annotation(
        Placement(visible = true, transformation(origin = {-20, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Translational.Sources.Speed speed_x(exact = true) annotation(
        Placement(visible = true, transformation(origin = {-60, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Product product_x annotation(
        Placement(visible = true, transformation(origin = {-94, 30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Math.Product product_y annotation(
        Placement(visible = true, transformation(origin = {-94, 50}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Math.Cos cos annotation(
        Placement(visible = true, transformation(origin = {-114, 26}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Math.Sin sin annotation(
        Placement(visible = true, transformation(origin = {-114, 46}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput x annotation(
        Placement(visible = true, transformation(origin = {130, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {128, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(visible = true, transformation(origin = {130, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {138, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput z annotation(
        Placement(visible = true, transformation(origin = {130, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {148, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput vx annotation(
        Placement(visible = true, transformation(origin = {130, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {136, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput vy annotation(
        Placement(visible = true, transformation(origin = {130, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {146, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput vz annotation(
        Placement(visible = true, transformation(origin = {130, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {156, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput ax annotation(
        Placement(visible = true, transformation(origin = {130, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {126, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput ay annotation(
        Placement(visible = true, transformation(origin = {130, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {136, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput az annotation(
        Placement(visible = true, transformation(origin = {130, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {146, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput qx annotation(
        Placement(visible = true, transformation(origin = {130, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {138, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput qy annotation(
        Placement(visible = true, transformation(origin = {130, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {148, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput qz annotation(
        Placement(visible = true, transformation(origin = {130, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {158, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput qw annotation(
        Placement(visible = true, transformation(origin = {130, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {168, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput wy annotation(
        Placement(visible = true, transformation(origin = {130, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {156, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput wx annotation(
        Placement(visible = true, transformation(origin = {130, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {146, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput wz annotation(
        Placement(visible = true, transformation(origin = {130, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {166, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles angles annotation(
        Placement(visible = true, transformation(origin = {30, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      differential_drive.Sensors.AbsoluteOdometryImu absoluteOdometryImu annotation(
        Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput w annotation(
        Placement(visible = true, transformation(origin = {-160, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-150, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput v annotation(
        Placement(visible = true, transformation(origin = {-160, 44}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-152, 38}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(world.frame_b, prismatic_x.frame_a) annotation(
        Line(points = {{-80, -10}, {-60, -10}}));
      connect(prismatic_x.frame_b, prismatic_y.frame_a) annotation(
        Line(points = {{-40, -10}, {-20, -10}}, color = {95, 95, 95}));
      connect(prismatic_y.frame_b, revolute_yaw.frame_a) annotation(
        Line(points = {{0, -10}, {20, -10}}));
      connect(revolute_yaw.frame_b, body.frame_a) annotation(
        Line(points = {{40, -10}, {60, -10}}, color = {95, 95, 95}));
      connect(body_shape.frame_a, body.frame_a) annotation(
        Line(points = {{62, -30}, {54, -30}, {54, -10}, {60, -10}}, color = {95, 95, 95}));
      connect(speed_w.flange, revolute_yaw.axis) annotation(
        Line(points = {{20, 80}, {30, 80}, {30, 0}}));
      connect(speed_y.flange, prismatic_y.axis) annotation(
        Line(points = {{-10, 50}, {-2, 50}, {-2, -4}}, color = {0, 127, 0}));
      connect(speed_x.flange, prismatic_x.axis) annotation(
        Line(points = {{-50, 30}, {-42, 30}, {-42, -4}}, color = {0, 127, 0}));
      connect(product_y.y, speed_y.v_ref) annotation(
        Line(points = {{-87.4, 50}, {-31.4, 50}}, color = {0, 0, 127}));
      connect(product_x.y, speed_x.v_ref) annotation(
        Line(points = {{-87.4, 30}, {-71.4, 30}}, color = {0, 0, 127}));
      connect(cos.y, product_x.u2) annotation(
        Line(points = {{-107.4, 26}, {-101.4, 26}}, color = {0, 0, 127}));
      connect(sin.y, product_y.u2) annotation(
        Line(points = {{-107.4, 46}, {-101.4, 46}}, color = {0, 0, 127}));
      connect(angles.frame_a, revolute_yaw.frame_b) annotation(
        Line(points = {{40, -40}, {48, -40}, {48, -10}, {40, -10}}));
      connect(cos.u, angles.angles[3]) annotation(
        Line(points = {{-121.2, 26}, {-129.2, 26}, {-129.2, -40}, {20.8, -40}}, color = {0, 0, 127}));
      connect(sin.u, angles.angles[3]) annotation(
        Line(points = {{-121.2, 46}, {-129.2, 46}, {-129.2, -40}, {20.8, -40}}, color = {0, 0, 127}));
      connect(absoluteOdometryImu.frame_a, revolute_yaw.frame_b) annotation(
        Line(points = {{60, -70}, {48, -70}, {48, -10}, {40, -10}}));
      connect(x, absoluteOdometryImu.position[1]) annotation(
        Line(points = {{130, 90}, {100, 90}, {100, -64}, {82, -64}}, color = {0, 0, 127}));
      connect(y, absoluteOdometryImu.position[2]) annotation(
        Line(points = {{130, 78}, {100, 78}, {100, -64}, {82, -64}}, color = {0, 0, 127}));
      connect(z, absoluteOdometryImu.position[3]) annotation(
        Line(points = {{130, 66}, {100, 66}, {100, -64}, {82, -64}}, color = {0, 0, 127}));
      connect(qx, absoluteOdometryImu.orientation[1]) annotation(
        Line(points = {{130, 54}, {104, 54}, {104, -66}, {82, -66}}, color = {0, 0, 127}));
      connect(qy, absoluteOdometryImu.orientation[2]) annotation(
        Line(points = {{130, 42}, {104, 42}, {104, -66}, {82, -66}}, color = {0, 0, 127}));
      connect(qz, absoluteOdometryImu.orientation[3]) annotation(
        Line(points = {{130, 30}, {104, 30}, {104, -66}, {82, -66}}, color = {0, 0, 127}));
      connect(qw, absoluteOdometryImu.orientation[4]) annotation(
        Line(points = {{130, 18}, {104, 18}, {104, -66}, {82, -66}}, color = {0, 0, 127}));
      connect(vx, absoluteOdometryImu.linear_velocity[1]) annotation(
        Line(points = {{130, 6}, {108, 6}, {108, -70}, {82, -70}}, color = {0, 0, 127}));
      connect(vy, absoluteOdometryImu.linear_velocity[2]) annotation(
        Line(points = {{130, -6}, {108, -6}, {108, -70}, {82, -70}}, color = {0, 0, 127}));
      connect(vz, absoluteOdometryImu.linear_velocity[3]) annotation(
        Line(points = {{130, -18}, {108, -18}, {108, -70}, {82, -70}}, color = {0, 0, 127}));
      connect(wx, absoluteOdometryImu.angular_velocity[1]) annotation(
        Line(points = {{130, -30}, {112, -30}, {112, -72}, {82, -72}}, color = {0, 0, 127}));
      connect(wy, absoluteOdometryImu.angular_velocity[2]) annotation(
        Line(points = {{130, -42}, {112, -42}, {112, -72}, {82, -72}}, color = {0, 0, 127}));
      connect(wz, absoluteOdometryImu.angular_velocity[3]) annotation(
        Line(points = {{130, -54}, {112, -54}, {112, -72}, {82, -72}}, color = {0, 0, 127}));
      connect(ax, absoluteOdometryImu.linear_acceleration[1]) annotation(
        Line(points = {{130, -66}, {116, -66}, {116, -76}, {82, -76}}, color = {0, 0, 127}));
      connect(ay, absoluteOdometryImu.linear_acceleration[2]) annotation(
        Line(points = {{130, -78}, {116, -78}, {116, -76}, {82, -76}}, color = {0, 0, 127}));
      connect(az, absoluteOdometryImu.linear_acceleration[3]) annotation(
        Line(points = {{130, -90}, {116, -90}, {116, -76}, {82, -76}}, color = {0, 0, 127}));
      connect(w, speed_w.w_ref) annotation(
        Line(points = {{-160, 80}, {-2, 80}}, color = {0, 0, 127}));
      connect(product_y.u1, v) annotation(
        Line(points = {{-102, 54}, {-160, 54}, {-160, 44}}, color = {0, 0, 127}));
      connect(product_x.u1, v) annotation(
        Line(points = {{-102, 34}, {-160, 34}, {-160, 44}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.02),
        Diagram);
    end kinematic_simulator;
  end Simulators;

  package Tests
    model simulator_test
      differential_drive.simulator simulator annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant speed_left(k = 0) annotation(
        Placement(visible = true, transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant speed_right(k = 0) annotation(
        Placement(visible = true, transformation(origin = {-70, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(speed_left.y, simulator.v_left) annotation(
        Line(points = {{-58, 30}, {-40, 30}, {-40, 8}, {-24, 8}}, color = {0, 0, 127}));
      connect(speed_right.y, simulator.v_right) annotation(
        Line(points = {{-58, -32}, {-40, -32}, {-40, -8}, {-24, -8}}, color = {0, 0, 127}));
    end simulator_test;
  end Tests;

  model simulator
    Modelica.Mechanics.Rotational.Sources.Speed speed_left annotation(
      Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    differential_drive.robot robot annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Sources.Speed speed_right annotation(
      Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_left annotation(
      Placement(visible = true, transformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput v_right annotation(
      Placement(visible = true, transformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput x annotation(
      Placement(visible = true, transformation(origin = {108, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput theta annotation(
      Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(speed_right.flange, robot.right_wheels) annotation(
      Line(points = {{-20, -30}, {-16, -30}, {-16, -4}, {-10, -4}}));
    connect(speed_left.flange, robot.left_wheels) annotation(
      Line(points = {{-20, 30}, {-16, 30}, {-16, 6}, {-10, 6}}));
    connect(speed_left.w_ref, v_left) annotation(
      Line(points = {{-42, 30}, {-120, 30}}, color = {0, 0, 127}));
    connect(speed_right.w_ref, v_right) annotation(
      Line(points = {{-42, -30}, {-120, -30}}, color = {0, 0, 127}));
    connect(y, robot.y) annotation(
      Line(points = {{110, 0}, {12, 0}}, color = {0, 0, 127}));
    connect(robot.x, x) annotation(
      Line(points = {{12, 4}, {40, 4}, {40, 40}, {108, 40}}, color = {0, 0, 127}));
    connect(robot.yaw, theta) annotation(
      Line(points = {{12, -4}, {40, -4}, {40, -40}, {110, -40}}, color = {0, 0, 127}));
    annotation(
      experiment(StartTime = 0, StopTime = 15, Tolerance = 1e-06, Interval = 0.03),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
      Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}})}));
  end simulator;

  model interaction_model
    import Modelica.Mechanics.MultiBody.Frames;
    import Modelica.Units.SI;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-106, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    parameter SI.Radius wheel_radius "Wheel radius";
    parameter Real Cx "Longitudinal stiffness";
    parameter Real Ca "Cornering stiffness";
    parameter SI.Velocity v_thd = 1.0E-6 "Linear velocity threshold";
    parameter SI.AngularVelocity w_thd = 1.0E-6 "Angular velocity threshold";
    parameter SI.Angle lambda_max = 3.14 "Maximum longitudinal slip value";
    // Contact force
    SI.Force f_wheel_0[3] "Contact force acting on wheel, resolved in world frame";
    SI.Force f_lat "Contact force acting on wheel in lateral direction";
    SI.Force f_long "Contact force acting on wheel in longitudinal direction";
    //protected
    Real e_axis_0[3] "Unit vector along wheel axis, resolved in world frame";
    SI.Position rContact_0[3] "Distance vector from wheel center to contact point, resolved in world frame";
    // Coordinate system at contact point
    Real e_n_0[3] "Unit vector in normal direction of road at contact point, resolved in world frame";
    Real e_lat_0[3] "Unit vector in lateral direction of wheel at contact point, resolved in world frame";
    Real e_long_0[3] "Unit vector in longitudinal direction of wheel at contact point, resolved in world frame";
    // Slip velocities
    SI.Velocity v_0[3] "Velocity of wheel center, resolved in world frame";
    SI.AngularVelocity w_0[3] "Angular velocity of wheel, resolved in world frame";
    // Slip
    Real lambda "Longitudinal wheel slip";
    Real alpha "Lateral wheel slip";
    // Utility vectors
    Real aux[3];
    Real pippo;
  equation
// Coordinate system at contact point (e_long_0, e_lat_0, e_n_0)
    e_n_0 = {0, 0, 1};
    e_axis_0 = Frames.resolve1(frame_a.R, {0, 1, 0});
    aux = cross(e_n_0, e_axis_0);
    e_long_0 = -aux/Modelica.Math.Vectors.length(aux);
    e_lat_0 = cross(-e_long_0, e_n_0);
// Wheel velocities and slip
    rContact_0 = {0, 0, -wheel_radius};
    v_0 = der(frame_a.r_0);
    w_0 = Frames.angularVelocity1(frame_a.R);
    lambda = smooth(0, if (abs(v_0*e_long_0) > v_thd) then (w_0*e_axis_0*wheel_radius - v_0*e_long_0)/(v_0*e_long_0) elseif (abs(w_0*e_axis_0*wheel_radius) > w_thd) then lambda_max else 0);
    alpha = -Modelica.Math.atan2(v_0*e_lat_0, v_0*e_long_0);
    pippo = smooth(0, if ((wheel_radius - frame_a.r_0*e_n_0) > 0) then (0.1*(wheel_radius - frame_a.r_0*e_n_0) - 0.1*v_0*e_n_0) else 0);
// Interaction forces
    f_long = Cx*lambda;
    f_lat = Ca*alpha;
    f_wheel_0 = f_lat*e_lat_0 + f_long*e_long_0 + 0*e_n_0;
// Force and torque balance at the wheel center
    zeros(3) = frame_a.f + Frames.resolve2(frame_a.R, f_wheel_0);
    zeros(3) = frame_a.t + Frames.resolve2(frame_a.R, cross(rContact_0, f_wheel_0));
    annotation(
      Diagram,
      Icon(graphics = {Text(textColor = {0, 0, 255}, extent = {{-150, 100}, {150, 140}}, textString = "%name"), Polygon(origin = {0, -79}, fillColor = {192, 191, 188}, fillPattern = FillPattern.Forward, points = {{-100, -1}, {-60, 19}, {-12, 5}, {32, 21}, {76, -9}, {100, -1}, {100, -21}, {-100, -21}, {-100, -1}}), Rectangle(extent = {{-100, 100}, {100, -100}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end interaction_model;

  model wheel
    import Modelica.Units.SI;
    Modelica.Mechanics.MultiBody.Visualizers.VoluminousWheel wheel_shape(rCurvature = 0.15, rRim = wheel_radius*0.5, rTire = wheel_radius, width = 0.1) annotation(
      Placement(visible = true, transformation(origin = {-20, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    differential_drive.interaction_model interaction_model(Ca = Ca, Cx = Cx, lambda_max = lambda_max, v_thd = v_thd, w_thd = w_thd, wheel_radius = wheel_radius) annotation(
      Placement(visible = true, transformation(origin = {20, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body wheel(animation = false, m = wheel_mass, r_CM = {0, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {-20, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-106, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Revolute wheel_joint(n = {0, 1, 0}, useAxisFlange = true) annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(
      Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter SI.Radius wheel_radius "Wheel radius";
    parameter SI.Mass wheel_mass "Wheel mass";
    parameter SI.Mass chassis_mass "Fraction of chassis mass on this wheel";
    parameter Real Cx "Longitudinal stiffness";
    parameter Real Ca "Cornering stiffness";
    parameter SI.Velocity v_thd = 1.0E-6 "Linear velocity threshold";
    parameter SI.AngularVelocity w_thd = 1.0E-6 "Angular velocity threshold";
    parameter SI.Angle lambda_max = 3.14 "Maximum longitudinal slip value";
  equation
    connect(wheel_shape.frame_a, wheel.frame_a) annotation(
      Line(points = {{-10, 60}, {0, 60}, {0, 30}, {-10, 30}}));
    connect(interaction_model.frame_a, wheel.frame_a) annotation(
      Line(points = {{9.4, 30}, {-10.6, 30}}));
    connect(frame_a, wheel_joint.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}}));
    connect(wheel_joint.frame_b, wheel.frame_a) annotation(
      Line(points = {{-40, 0}, {0, 0}, {0, 30}, {-10, 30}}));
    connect(flange_a, wheel_joint.axis) annotation(
      Line(points = {{-100, 40}, {-50, 40}, {-50, 10}}));
    annotation(
      Icon(graphics = {Polygon(lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Sphere, points = {{-40, 80}, {-20, 90}, {15, 90}, {40, 85}, {58.776, 73.91}, {70.456, 56.568}, {74.951, 44.383}, {78.26, 30.614}, {80.302, 15.68}, {81, 0}, {81, 0}, {80.302, -15.68}, {78.26, -30.614}, {74.951, -44.383}, {70.456, -56.568}, {58.776, -73.91}, {40, -85}, {15, -90}, {-20, -90}, {-40, -80}, {-48.776, -73.91}, {-60.456, -56.568}, {-64.951, -44.383}, {-68.26, -30.614}, {-70.302, -15.68}, {-71, 0}, {-71, 0}, {-70.302, 15.68}, {-68.26, 30.614}, {-64.951, 44.383}, {-60.456, 56.568}, {-48.776, 73.91}, {-40, 80}}, smooth = Smooth.Bezier), Polygon(lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{1, 0}, {0.302, 15.68}, {-1.74, 30.614}, {-5.049, 44.383}, {-9.544, 56.568}, {-21.224, 73.91}, {-35, 80}, {-48.776, 73.91}, {-60.456, 56.568}, {-64.951, 44.383}, {-68.26, 30.614}, {-70.302, 15.68}, {-71, 0}, {-70.302, -15.68}, {-68.26, -30.614}, {-64.951, -44.383}, {-60.456, -56.568}, {-48.776, -73.91}, {-35, -80}, {-21.224, -73.91}, {-9.544, -56.568}, {-5.049, -44.383}, {-1.74, -30.614}, {0.302, -15.68}, {1, 0}}, smooth = Smooth.Bezier), Polygon(lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, points = {{-12.5, 0}, {-14.213, -19.134}, {-19.09, -35.355}, {-26.39, -46.194}, {-35, -50}, {-43.61, -46.194}, {-50.91, -35.355}, {-55.787, -19.134}, {-57.5, 0}, {-55.787, 19.134}, {-50.91, 35.355}, {-43.61, 46.194}, {-35, 50}, {-26.39, 46.194}, {-19.09, 35.355}, {-14.213, 19.134}, {-12.5, 0}}, smooth = Smooth.Bezier), Text(textColor = {0, 0, 255}, extent = {{-150, 100}, {150, 140}}, textString = "%name")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end wheel;

  model robot_old
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_rear_left_translation(animation = false, r = {0.25, 0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {-50, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_rear_right_translation(animation = false, r = {0.25, -0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_rear_left(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {-90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_rear_right(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_front_left(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_front_right(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 0.241666667, I_22 = 1.666666667, I_33 = 1.841666667, angles_fixed = false, animation = false, m = 40, r_CM = {0, 0, 0}, w_0_fixed = false, z_0_fixed = false) annotation(
      Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_x(animation = false, s(fixed = true), useAxisFlange = true, v(fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_y(animation = false, n = {0, 1, 0}, s(fixed = true), useAxisFlange = true, v(fixed = true)) annotation(
      Placement(visible = true, transformation(origin = {-10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute_yaw(useAxisFlange = true) annotation(
      Placement(visible = true, transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_front_left_translation(animation = false, r = {-0.25, 0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_front_right_translation(animation = false, r = {-0.25, -0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Visualizers.FixedShape chassis_shape(height = 0.1, length = 0.7, r_shape = {-0.35, 0, 0}, width = 0.25) annotation(
      Placement(visible = true, transformation(origin = {-30, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a left_wheels annotation(
      Placement(visible = true, transformation(origin = {-110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a right_wheels annotation(
      Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput x annotation(
      Placement(visible = true, transformation(origin = {-118, -24}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {-118, -44}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput yaw annotation(
      Placement(visible = true, transformation(origin = {-118, -64}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Sensors.RelAngleSensor yaw_sensor annotation(
      Placement(visible = true, transformation(origin = {16, -64}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Sensors.RelPositionSensor x_sensor annotation(
      Placement(visible = true, transformation(origin = {-22, -44}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Sensors.RelPositionSensor y_sensor annotation(
      Placement(visible = true, transformation(origin = {-62, -24}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    connect(wheel_rear_left.frame_a, wheel_rear_left_translation.frame_b) annotation(
      Line(points = {{-79.4, 40}, {-59.4, 40}}));
    connect(wheel_rear_right.frame_a, wheel_rear_right_translation.frame_b) annotation(
      Line(points = {{79.4, 40}, {59.4, 40}}));
    connect(wheel_rear_right_translation.frame_a, body.frame_a) annotation(
      Line(points = {{40, 40}, {0, 40}, {0, 70}}));
    connect(wheel_rear_left_translation.frame_a, body.frame_a) annotation(
      Line(points = {{-40, 40}, {0, 40}, {0, 70}}));
    connect(prismatic_x.frame_b, prismatic_y.frame_a) annotation(
      Line(points = {{-40, -90}, {-20, -90}}));
    connect(prismatic_y.frame_b, revolute_yaw.frame_a) annotation(
      Line(points = {{0, -90}, {20, -90}}, color = {95, 95, 95}));
    connect(revolute_yaw.frame_b, body.frame_a) annotation(
      Line(points = {{40, -90}, {50, -90}, {50, -20}, {0, -20}, {0, 70}}));
    connect(wheel_front_left.frame_a, wheel_front_left_translation.frame_b) annotation(
      Line(points = {{-79.4, 0}, {-59.4, 0}}, color = {95, 95, 95}));
    connect(wheel_front_right.frame_a, wheel_front_right_translation.frame_b) annotation(
      Line(points = {{79.4, 0}, {59.4, 0}}));
    connect(wheel_front_left_translation.frame_a, body.frame_a) annotation(
      Line(points = {{-40, 0}, {0, 0}, {0, 70}}, color = {95, 95, 95}));
    connect(wheel_front_right_translation.frame_a, body.frame_a) annotation(
      Line(points = {{40, 0}, {0, 0}, {0, 70}}, color = {95, 95, 95}));
    connect(left_wheels, wheel_rear_left.flange_a) annotation(
      Line(points = {{-110, 70}, {-70, 70}, {-70, 48}, {-80, 48}}));
    connect(left_wheels, wheel_front_left.flange_a) annotation(
      Line(points = {{-110, 70}, {-70, 70}, {-70, 8}, {-80, 8}}));
    connect(right_wheels, wheel_rear_right.flange_a) annotation(
      Line(points = {{110, 70}, {70, 70}, {70, 48}, {80, 48}}));
    connect(right_wheels, wheel_front_right.flange_a) annotation(
      Line(points = {{110, 70}, {70, 70}, {70, 8}, {80, 8}}));
    connect(chassis_shape.frame_a, body.frame_a) annotation(
      Line(points = {{-20, 60}, {0, 60}, {0, 70}}, color = {95, 95, 95}));
    connect(world.frame_b, prismatic_x.frame_a) annotation(
      Line(points = {{-80, -90}, {-60, -90}}, color = {95, 95, 95}));
    connect(yaw_sensor.phi_rel, yaw) annotation(
      Line(points = {{6, -64}, {-118, -64}}, color = {0, 0, 127}));
    connect(yaw_sensor.flange_b, revolute_yaw.support) annotation(
      Line(points = {{16, -74}, {24, -74}, {24, -80}}));
    connect(yaw_sensor.flange_a, revolute_yaw.axis) annotation(
      Line(points = {{16, -54}, {30, -54}, {30, -80}}));
    connect(x_sensor.flange_b, prismatic_y.support) annotation(
      Line(points = {{-22, -54}, {-14, -54}, {-14, -84}}, color = {0, 127, 0}));
    connect(x_sensor.flange_a, prismatic_y.axis) annotation(
      Line(points = {{-22, -34}, {-2, -34}, {-2, -84}}, color = {0, 127, 0}));
    connect(x_sensor.s_rel, y) annotation(
      Line(points = {{-32, -44}, {-118, -44}}, color = {0, 0, 127}));
    connect(y_sensor.flange_b, prismatic_x.support) annotation(
      Line(points = {{-62, -34}, {-54, -34}, {-54, -84}}, color = {0, 127, 0}));
    connect(y_sensor.flange_a, prismatic_x.axis) annotation(
      Line(points = {{-62, -14}, {-42, -14}, {-42, -84}}, color = {0, 127, 0}));
    connect(y_sensor.s_rel, x) annotation(
      Line(points = {{-72, -24}, {-118, -24}}, color = {0, 0, 127}));
    annotation(
      experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-6, Interval = 0.01),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
      Icon(graphics = {Text(textColor = {0, 0, 255}, extent = {{-150, 100}, {150, 140}}, textString = "%name"), Rectangle(extent = {{-100, 100}, {100, -100}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end robot_old;

  model robot
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_rear_left_translation(animation = false, r = {0.25, 0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {-50, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_rear_right_translation(animation = false, r = {0.25, -0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_rear_left(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {-90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_rear_right(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_front_left(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    differential_drive.wheel wheel_front_right(Ca = 50, Cx = 50, chassis_mass = 10, wheel_mass = 2, wheel_radius = 0.2) annotation(
      Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 0.241666667, I_22 = 1.666666667, I_33 = 1.841666667, angles_fixed = false, animation = false, m = 40, r_0(start = {0, 0, 0.5}), r_CM = {0, 0, 0}, w_0_fixed = false, z_0_fixed = false) annotation(
      Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_front_left_translation(animation = false, r = {-0.25, 0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation wheel_front_right_translation(animation = false, r = {-0.25, -0.2, 0}) annotation(
      Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Visualizers.FixedShape chassis_shape(height = 0.1, length = 0.7, r_shape = {-0.35, 0, 0}, width = 0.25) annotation(
      Placement(visible = true, transformation(origin = {-30, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a left_wheels annotation(
      Placement(visible = true, transformation(origin = {-110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a right_wheels annotation(
      Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput x annotation(
      Placement(visible = true, transformation(origin = {-118, -24}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {-118, -44}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput yaw annotation(
      Placement(visible = true, transformation(origin = {-118, -64}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Constant const annotation(
      Placement(visible = true, transformation(origin = {-54, -44}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(wheel_rear_left.frame_a, wheel_rear_left_translation.frame_b) annotation(
      Line(points = {{-79.4, 40}, {-59.4, 40}}));
    connect(wheel_rear_right.frame_a, wheel_rear_right_translation.frame_b) annotation(
      Line(points = {{79.4, 40}, {59.4, 40}}));
    connect(wheel_rear_right_translation.frame_a, body.frame_a) annotation(
      Line(points = {{40, 40}, {0, 40}, {0, 70}}));
    connect(wheel_rear_left_translation.frame_a, body.frame_a) annotation(
      Line(points = {{-40, 40}, {0, 40}, {0, 70}}));
    connect(wheel_front_left.frame_a, wheel_front_left_translation.frame_b) annotation(
      Line(points = {{-79.4, 0}, {-59.4, 0}}, color = {95, 95, 95}));
    connect(wheel_front_right.frame_a, wheel_front_right_translation.frame_b) annotation(
      Line(points = {{79.4, 0}, {59.4, 0}}));
    connect(wheel_front_left_translation.frame_a, body.frame_a) annotation(
      Line(points = {{-40, 0}, {0, 0}, {0, 70}}, color = {95, 95, 95}));
    connect(wheel_front_right_translation.frame_a, body.frame_a) annotation(
      Line(points = {{40, 0}, {0, 0}, {0, 70}}, color = {95, 95, 95}));
    connect(left_wheels, wheel_rear_left.flange_a) annotation(
      Line(points = {{-110, 70}, {-70, 70}, {-70, 48}, {-80, 48}}));
    connect(left_wheels, wheel_front_left.flange_a) annotation(
      Line(points = {{-110, 70}, {-70, 70}, {-70, 8}, {-80, 8}}));
    connect(right_wheels, wheel_rear_right.flange_a) annotation(
      Line(points = {{110, 70}, {70, 70}, {70, 48}, {80, 48}}));
    connect(right_wheels, wheel_front_right.flange_a) annotation(
      Line(points = {{110, 70}, {70, 70}, {70, 8}, {80, 8}}));
    connect(chassis_shape.frame_a, body.frame_a) annotation(
      Line(points = {{-20, 60}, {0, 60}, {0, 70}}, color = {95, 95, 95}));
    connect(y, const.y) annotation(
      Line(points = {{-118, -44}, {-64, -44}}, color = {0, 0, 127}));
    connect(x, const.y) annotation(
      Line(points = {{-118, -24}, {-80, -24}, {-80, -44}, {-64, -44}}, color = {0, 0, 127}));
    connect(yaw, const.y) annotation(
      Line(points = {{-118, -64}, {-80, -64}, {-80, -44}, {-64, -44}}, color = {0, 0, 127}));
    annotation(
      experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-6, Interval = 0.01),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
      Icon(graphics = {Text(textColor = {0, 0, 255}, extent = {{-150, 100}, {150, 140}}, textString = "%name"), Rectangle(extent = {{-100, 100}, {100, -100}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end robot;
  annotation(
    uses(Modelica(version = "4.0.0")));
end differential_drive;