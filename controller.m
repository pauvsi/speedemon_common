function x = controller(carLength, steering_angle, velocity)
    radius = carLength/sin(steering_angle);
    a_centripetal = velocity*velocity/radius;
end