require_xpr64;
reg_t lhs = zext32(RS1);
reg_t rhs = zext32(RS2);
if(rhs == 0)
  WRITE_RD(sext32(lhs));
else
  WRITE_RD(sext32(lhs % rhs));
