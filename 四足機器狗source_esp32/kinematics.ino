/* - ::: KINEMATICS :::

  This file is part of warp_kinematics. 該文件是warp_kinematics的一部分。
  [hardware] This file manages the basic hardware functions. [硬體] 這個文件管理基本的硬體功能。

  [BACK] [LEFT], LOWER JOINT (0, 0) : servo00, [後] [左], 下關節 (0, 0) : 伺服00,
  UPPER JOINT (0, 1) : servo02, SHLDR JOINT (0, 2) :servo04 上關節（0，1）：伺服02，肩關節（0，2）：伺服04。

  [FRONT][LEFT], LOWER JOINT (1, 0) : servo15, [前][左]，下交點（1，0）：伺服15。
  UPPER JOINT (1, 1) : servo13, SHLDR JOINT (1, 2) :servo05 上關節（1，1）：伺服13，肩關節（1，2）：伺服05

  [FRONT][RIGHT], LOWER JOINT (2, 0) : servo14, [前][右]，下關節（2，0）：伺服14。
  UPPER JOINT (2, 1) : servo12, SHLDR JOINT (2, 2) :servo07 上關節（2，1）：伺服12，肩關節（2，2）：伺服07

  [BACK] [RIGHT], LOWER JOINT (3, 0) : servo01, [後][右]，下關節（3，0）：伺服01。
  UPPER JOINT (3, 1) : servo03, SHLDR JOINT (3, 2) :servo06 上關節（3，1）：伺服03，肩關節（3，2）：伺服06

*/

/*
  ==============================
  KINEMATICS PARAMETERS 運動學參數
  ==============================
*/

//: this array stores the inverse direction(relative to the body) of each parent joint. 
//:這個數組儲存每個父關節的反方向（相對於身體）。
const float l_inv[4][2] = {
  { +1.f, -1.f}, // ## {dir, dir}
  { -1.f, -1.f}, // ## {dir, dir}
  { -1.f, +1.f}, // ## {dir, dir}
  { +1.f, +1.f}  // ## {dir, dir}
};

/*
  ::: HANDLE LOOP ::: ::: 處理循環 :::
*/

float c[4];
float c_iter[4];

boolean c_inv[4];

float stored_0x = 0.f;
void handle_kinematics(datatypes::Vector2D _dir, float _turn, float _height, float period) {
  for (int l = 0; l < 4; l++) {
    float base = c_base(90) + base_offset[l] + _height; //> stores the base of each leg 儲存每條腿的底座

    datatypes::Vector2D dir = {
      precision + _dir.x + (_turn) * l_inv[l][1],
      precision + _dir.y + (_turn) * l_inv[l][0]
    };
    count_c(l, dir, period); //> calls the clock function 調用時鐘函數

    datatypes::Vector2D rDir = c_direction_ratio(dir); //: direction ratio calculation function 方向比計算函數
    datatypes::Vector vector = {0, 0, 0}; //> default leg coordinates 預設腿部坐標

    //: these functions run for each leg and return a 3 dimensional vector that stores the desired leg position in cartesian coordinates
	//: 這些函數針對每條腿運行，並返回一個3維向量，該向量以笛卡爾坐標存儲所需的腿部位置。
    if (state == 1 && (abs(dir.x) > precision || abs(dir.y) > precision))
      vector = trot_gait_func({rDir.x * c[l], l_inv[l][1] * rDir.y * c[l]},
                              dir, boolean(l % 2) ^ boolean(c_inv[l] % 2));
    else if (state == 2)
      vector = yaw_axis(l, _turn);
    else if (state == 3)
      vector = pitch_roll_axis(l, base, {0, _dir.x , _dir.y});
    else if (state == 4) {
      vector = yaw_axis(l, stored_0x);
      if (abs(stored_0x + _turn / 4.f) < 32.f)
        stored_0x = inter(stored_0x, stored_0x + _turn / 4.f, 0.5f);
    }

    //: this datatype stores three values which correspond to the three joint angles of each leg,
	//: 該數據類型儲存三個數值，對應於每條腿的三個關節角度。
    /// the 3 dimensional vector is converted through the k_model function into these three angles.
	/// 三維向量通過k_model函數轉換為這三個角度。
    datatypes::Rotator cRot = k_model(vrt_offset, hrz_offset, base,
                                      0, 0, vector);
    set_leg(l, cRot); 
	//> this function sets the three servos of each leg to the calculated value 
	//>此函數將每個腿的三個舵機設置為計算值
  }
}

void count_c(int inst, datatypes::Vector2D dir, float period) {
  float w0 = step_extent.x * mm / (2  / max(abs(dir.x), abs(dir.y)));
  float a0 = (w0 * 2) * (c_iter[inst] / round(frequency / period)) - w0;

  c[inst] = a0;
  c_iter[inst] += 1.f;

  if (c_iter[inst] > round(frequency / period)) {
    c[inst] = -w0;
    c_iter[inst] = 1.f;

    c_inv[inst] = !c_inv[inst];
  }
}

/*
  ::: [KINEMATICS] FUNCTIONS ::: ::: [運動學]函數 :::
*/

/*
     ::: GAIT FUNCTIONS ::: ::: 步態函數 :::
*/

//: trot function //: 小跑函數
datatypes::Vector trot_gait_func(datatypes::Vector2D c0, datatypes::Vector2D dir, boolean inv) {
  float w0 = step_extent.x * mm / 2 * dir.x;
  float l0 = step_extent.y * mm * 4 * dir.y;
  float h0 = step_extent.z * mm;

  if (inv == false)
    c0 = { -c0.x, -c0.y};

  float h1 = sqrt(abs((1 - sq(c0.x / w0) - sq(c0.y / l0)) * sq(h0)));
  return {c0.x / mm, c0.y / mm, h1 / mm * int(inv)};
}

/*
  ::: TRIGONOMETRIC FUNCTIONS ::: ::: 三角函數 :::
*/

//: base calculation function 基準計算函數
float c_base(float angle1) {
  return sin(radians(angle1 / 2)) * bone_length * 2;
}

//: pitch-roll axis function 俯仰滾動軸功能
datatypes::Vector pitch_roll_axis(int leg, float base, datatypes::Rotator sRot) {
  float w0 = body_transform.scl.x / 2 * l_inv[leg][0] + p_joint_origin[leg].x - vrt_offset;
  float l0 = body_transform.scl.z / 2 + hrz_offset;

  float C0 = radians(sRot.pitch);
  float C1 = radians(sRot.roll) * l_inv[leg][1];

  float a0 = sin(C0) * w0;
  float a1 = sin(C1) * l0;

  float d0 = (1 - cos(C0)) * -w0;
  float d1 = (1 - cos(C1)) * l0;

  float var0 = sqrt(sq(base + a0) + sq(d0));
  C0 += asin(d0 / var0);

  float b0 = cos(C0) * var0;
  float c0 = sin(C0) * var0;

  float var1 = sqrt(sq(b0 - a1) + sq(d1));
  C1 += asin(d1 / var1);

  float b1 = cos(C1) * var1;
  float c1 = sin(C1) * var1;

  return {c0, c1, base - b1};
}

//: yaw axis function 偏航軸函數
datatypes::Vector yaw_axis(int leg, float yaw) {
  float x = body_transform.scl.x / 2 - abs(p_joint_origin[leg].x) - vrt_offset * l_inv[leg][0];
  float y = body_transform.scl.z / 2 + hrz_offset;
  float radius = sqrt(sq(x) + sq(y));
  float angle = asin(y / radius) - radians(yaw) * l_inv[leg][0] * l_inv[leg][1];

  float rX = (x - cos(angle) * radius) * l_inv[leg][0];
  float rY = sin(angle) * radius - y;
  return {rX, rY, 0};
}

//: direction ratio calculation function 方向比計算函數
datatypes::Vector2D c_direction_ratio(datatypes::Vector2D dir) {
  float dirX = dir.x / max(abs(dir.x), abs(dir.y));
  float dirY = dir.y / max(abs(dir.x), abs(dir.y));
  return {dirX, dirY};
}

//: inverse kinematic algorithm 逆運動學算法
datatypes::Rotator k_model(float x0, float y0, float z0,
                           float pitch_offset, float roll_offset, datatypes::Vector vec) {
  float x = x0 + vec.x,
        y = y0 + vec.y,
        z = z0 - vec.z;

  float b0 = sqrt(sq(x) + sq(y));
  float h0 = sqrt(sq(b0) + sq(z));

  float a0 = degrees(atan(x / z));
  float a1 = degrees(atan(y / z));

  return c_triangle(a0 + pitch_offset, a1 + roll_offset, h0);
}

//: final triangle calculation function 最終三角形計算函數
datatypes::Rotator c_triangle(float a0, float a1, float b0) {
  float angle1 = a1;
  float angle3 = degrees(asin((b0 / 2.0) / bone_length)) * 2;
  float angle2 = angle3 / 2 + a0;

  return {angle1, angle2, angle3};
}
