#include <iostream>

#include <ccd/ccd.h>
#include <ccd/quat.h>

struct Box {
  ccd_vec3_t pos;
  ccd_quat_t quat;

  ccd_real_t x, y, z;
};

void support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *v) {
  Box *box = (Box *) _obj;

  ccd_vec3_t dir;
  ccd_quat_t qinv;

  ccdVec3Copy(&dir, _dir);
  ccdQuatInvert2(&qinv, &box->quat);
  ccdQuatRotVec(&dir, &qinv);

  ccdVec3Set(v,
      ccdSign(ccdVec3X(&dir)) * box->x * CCD_REAL(0.5),
      ccdSign(ccdVec3Y(&dir)) * box->y * CCD_REAL(0.5),
      ccdSign(ccdVec3Z(&dir)) * box->z * CCD_REAL(0.5));

  ccdQuatRotVec(v, &box->quat);
  ccdVec3Add(v, &box->pos);
}

int main(void) {
  Box box1;

  box1.x = 2;
  box1.y = 2;
  box1.z = 2;

  ccdVec3Set(&box1.pos, 0, 0, 0);

  Box box2;

  box2.x = 2;
  box2.y = 2;
  box2.z = 2;

  ccdVec3Set(&box2.pos, 0, 0, 0);

  ccdQuatSet(&box1.quat, 0, 0, 0, 1);
  ccdQuatSet(&box2.quat, 0, 0, 0, 1);

  ccd_t ccd;
  CCD_INIT(&ccd);

  ccd.support1 = support;
  ccd.support2 = support;
  ccd.max_iterations = 100;

  int intersect = ccdGJKIntersect(&box1, &box2, &ccd);

  std::cout << intersect << std::endl;

  return 0;
}
