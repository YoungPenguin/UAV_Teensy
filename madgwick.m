beta =0.041;
invSampleFreq = 1.0 /512.0;
ax = 0;
ay = 0;
az = 0;
mx = 0;
my = 0;
mz = 0;
gx = 0;
gy = 0;
gz = 0;
q0 = 1.0;
q1 = 0.0;
	q2 = 0.0;
	q3 = 0.0;
    
    roll = zeros(100,1);
pitch = zeros(100,1);
yaw = zeros(100,1);
    
for i=1:100


qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    
recipNorm = norm([ax ay az]);
		ax = ax / recipNorm;
		ay = ay / recipNorm;
		az = az / recipNorm;
recipNorm = norm([mx my mz]);
		mx = mx /recipNorm;
		my =my / recipNorm;
		mz =mz / recipNorm;
        
    
    xxx  = 2.0 * q0 * mx;
		yyy = 2.0 * q0 * my;
		zzz = 2.0 * q0 * mz;
		x = 2.0 * q1 * mx;
		qq02 = 2.0 * q0;
		qq12 = 2.0 * q1;
		qq2 = 2.0 * q2;
		qq23 = 2.0 * q3;
		qq02q2 = 2.0 * q0 * q2;
		qqqq23 = 2.0 * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		qqq2 = q2 * q2;
		qqq23 = q2 * q3;
		q3q3 = q3 * q3;
        
        
        hx = mx * q0q0 - yyy * q3 + zzz * q2 + mx * q1q1 +  qq12 * my * q2 +  qq12 * mz * q3 - mx * qqq2 - mx * q3q3;
		hy = xxx  * q3 + my * q0q0 - zzz * q1 + x * q2 - my * q1q1 + my * qqq2 +  qq2 * mz * q3 - my * q3q3;
		bx2 = sqrt(hx * hx + hy * hy);
		bz2 = -xxx  * q2 + yyy * q1 + mz * q0q0 +  x * q3 - mz * q1q1 +  qq2 * my * q3 - mz * qqq2 + mz * q3q3;
		bx4 = 2.0 * bx2;
		bz4 = 2.0 * bz2;
        
      s0 = - qq2 * (2.0 * q1q3 -  qq02q2 - ax) +  qq12 * (2.0 * q0q1 +  qqqq23 - ay) -  bz2 * q2 * ( bx2 * (0.5 - qqq2 - q3q3) +  bz2 * (q1q3 - q0q2) - mx) + (- bx2 * q3 +  bz2 * q1) * ( bx2 * (q1q2 - q0q3) +  bz2 * (q0q1 + qqq23) - my) +  bx2 * q2 * ( bx2 * (q0q2 + q1q3) +  bz2 * (0.5 - q1q1 - qqq2) - mz);
		s1 =  qq23 * (2.0 * q1q3 -  qq02q2 - ax) +  qq02 * (2.0 * q0q1 +  qqqq23 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * qqq2 - az) +  bz2 * q3 * ( bx2 * (0.5 - qqq2 - q3q3) +  bz2 * (q1q3 - q0q2) - mx) + ( bx2 * q2 +  bz2 * q0) * ( bx2 * (q1q2 - q0q3) +  bz2 * (q0q1 + qqq23) - my) + ( bx2 * q3 -  bz4 * q1) * ( bx2 * (q0q2 + q1q3) +  bz2 * (0.5 - q1q1 - qqq2) - mz);
		s2 = - qq02 * (2.0 * q1q3 -  qq02q2 - ax) +  qq23 * (2.0 * q0q1 +  qqqq23 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * qqq2 - az) + (- bx4 * q2 -  bz2 * q0) * ( bx2 * (0.5 - qqq2 - q3q3) +  bz2 * (q1q3 - q0q2) - mx) + ( bx2 * q1 +  bz2 * q3) * ( bx2 * (q1q2 - q0q3) +  bz2 * (q0q1 + qqq23) - my) + ( bx2 * q0 -  bz4 * q2) * ( bx2 * (q0q2 + q1q3) +  bz2 * (0.5 - q1q1 - qqq2) - mz);
		s3 =  qq12 * (2.0 * q1q3 -  qq02q2 - ax) +  qq2 * (2.0 * q0q1 +  qqqq23 - ay) + (- bx4 * q3 +  bz2 * q1) * ( bx2 * (0.5 - qqq2 - q3q3) +  bz2 * (q1q3 - q0q2) - mx) + (- bx2 * q0 +  bz2 * q2) * ( bx2 * (q1q2 - q0q3) +  bz2 * (q0q1 + qqq23) - my) +  bx2 * q1 * ( bx2 * (q0q2 + q1q3) +  bz2 * (0.5 - q1q1 - qqq2) - mz);
		recipNorm = norm([s0 s1 s2 s3]);
		s0 = s0/recipNorm;
		s1 = s1/recipNorm;
		s2 = s2/recipNorm;
		s3 = s3/recipNorm;
  
        qDot1 =qDot1- beta * s0;
		qDot2 =qDot2- beta * s1;
		qDot3 = qDot3-beta * s2;
		qDot4 =qDot4- beta * s3;
        
    q0 =q0+ qDot1 * invSampleFreq;
	q1 = q1+qDot2 * invSampleFreq;
	q2 = q2+qDot3 * invSampleFreq;
	q3 =q3+ qDot4 * invSampleFreq;
    
roll(i) = atan((q0*q1 + q2*q3)/( 0.5 - q1*q1 - q2*q2));
pitch(i) = asin(-2.0 * (q1*q3 - q0*q2));
yaw(i) = atan((q1*q2 + q0*q3)/ (0.5 - q2*q2 - q3*q3));
end
