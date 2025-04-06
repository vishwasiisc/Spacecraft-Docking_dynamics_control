function q = add_quaternion(q2,q1)

q1_vec = [q1(1),q1(2),q1(3),q1(4)]';


M = [ q2(1), -q2(2), -q2(3), -q2(4);
     -q2(2),  q2(1),  q2(4), -q2(3);
     -q2(3), -q2(4),  q2(1), -q2(2);
      q2(4), -q2(3), -q2(2),  q2(1) ];

q = M*q1_vec;

if q(1) < 0
    %q = -q;
end

end