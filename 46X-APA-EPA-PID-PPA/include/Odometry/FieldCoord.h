#include "vex.h"
struct FieldCoord{
  PVector pos;
  double angle;
public:
  FieldCoord(){
    pos = PVector();
    angle = 0.0;
  }
  FieldCoord(PVector& v, double& a){
    pos = v;
    angle = a;
  }
  FieldCoord(PVector&& v, double&& a){
    pos = v;
    angle = a;
  }
  FieldCoord& set(PVector v, double a){
    pos = v;
    angle = a;
    CHAIN
  }
  bool operator!=(FieldCoord& f){
    return f.pos != pos || f.angle != angle;
  }
  bool operator==(FieldCoord& f){
    return f.pos == pos && f.angle == angle;
  }
  bool en = false;
  FieldCoord& operator+= (FieldCoord& f){
    if(en){
      cout << pos << "\n" << f.pos << endl;
    }
    this->pos += f.pos;
    if(en){
      cout << pos << "\n" << f.pos << endl << endl;
    }
    this->angle += f.angle;
    CHAIN
  }
  FieldCoord operator-(FieldCoord& p){
    return FieldCoord(pos - p.pos, angle - p.angle);
  }
  FieldCoord operator-(){
    return FieldCoord(-pos, angle + 180);
  }
};
ostream& operator<< (ostream& cout, FieldCoord& v){
  cout << VECT_OUT(v.pos) << ", " << v.angle;
  return cout;
}
ostream& operator<<(ostream& cout, FieldCoord&& v){
  return operator<<(cout, v);
}