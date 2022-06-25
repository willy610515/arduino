#define mm 0.1 /// millimeter 毫米

int sign(float num) {
  return int(num >= 0) - int(num < 0);
}

class datatypes {
  public:
    /* ::: STEP, DATA TYPE ::: */ /* ::: 步階，數據類型 ::: */
    struct Step {
      float base;  //
      float angle;
    };

    /* ::: VECTOR, DATA TYPE ::: */ /* ::: 向量，數據類型 ::: */
    struct Vector {
      float x;
      float y;
      float z;
    };

    /* ::: 2D VECTOR, DATA TYPE ::: */ /* ::: 2D 向量，數據類型 ::: */
    struct Vector2D {
      float x;
      float y;
    };

    /* ::: ROTATOR, DATA TYPE ::: */ /* ::: 旋轉器，數據類型 ::: */
    struct Rotator {
      float yaw;
      float pitch;
      float roll;
    };

    /* ::: TRANFORM, DATA TYPE ::: */ /* ::: 轉換，數據類型 ::: */
    struct Transform {
      Vector pos;
      Rotator rot;
      Vector scl;
    };
};
