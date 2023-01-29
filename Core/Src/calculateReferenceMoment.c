double Jn     = 0;
double Kvt    = 0;
double Ksh    = 0;
double Jdv_im = 0;
double q_im   = 0;
double Ce_im  = 0;

void setCoefficientReferenceMoment(double JnUser, double KvtUser,
                                   double KshUser, double Jdv_imUser, double q_imUser, double Ce_imUser)
{
   Jn     = JnUser;
   Kvt    = KvtUser;
   Ksh    = KshUser;
   Jdv_im = Jdv_imUser;
   q_im   = q_imUser;
   Ce_im  = Ce_imUser;
}


double calculateReferenceMoment(double currentAngle, double currentSpeedAngle,
                                double currentAccelerationAngle)
{
   return Jn * currentAccelerationAngle + Kvt * currentSpeedAngle
          + Ksh * currentAngle
          - Jdv_im * q_im * q_im * currentAccelerationAngle
          + Ce_im * q_im * currentSpeedAngle;
}
