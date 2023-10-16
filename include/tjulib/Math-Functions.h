#pragma once
#include <vector>

namespace tjulib{

  struct graphPoint
  {
    float x, y;
  };

  class Math
  {
    private:
      const float rad_In;
    public:
      // CONVERSIONS (DISTANCE)
      Math(float rad_In):rad_In(rad_In){};
      float degToInch(float deg);
      float inchToDeg(float inch);

      // CONVERSIONS (ANGLE)
      float getRadians(float deg);
      float getDeg(float rad);

      // HELPER FUNCTIONS
      float getHeading(float angle);
      float angleWrap(float angle);
      float compressAngle(float startAngle, float angle);
      float clip(float number, float min, float max);
      int optimalTurnSide(float currentA, float targetA);

      // GEOMETRY FUNCTIONS
      float dist(graphPoint point1, graphPoint point2);
      bool linePoint(graphPoint linePoint1, graphPoint linePoint2, graphPoint point);
      bool pointCircle(graphPoint point, graphPoint circleCenter, float cr);
      bool lineCircle(graphPoint linePoint1, graphPoint linePoint2, graphPoint circleCenter, float r);

      static constexpr float velocityToVoltage = 12000/200;


      // PURE PURSUIT
      std::vector<graphPoint> lineCircleIntersection(graphPoint circleCenter, float radius,
                                                            graphPoint linePoint1, graphPoint linePoint2);
  };
};

