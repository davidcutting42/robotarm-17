#ifndef MyTypes_h
#define MyTypes_h
#endif

typedef struct {
  float x; 
  float y;
  int lhrh;
  float base;
  float dangle;
  float saangle;
  float sbangle;
  int actiontypexy; // 0 = slow, 1 = fast
  int actiontypelift; // 0 = slow, 1 = fast
  int actiontypeservos; // 0 = slow, 1 = fast
} waypoint;
