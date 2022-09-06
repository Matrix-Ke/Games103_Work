using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity
    float friction = 0.6f;

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    //经验数据,可以修改
    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;


    float restitution = 0.5f;                   // for collision
    float gravity = 9.8f;                          //gravity

    Vector3 V_1 = new Vector3(0, 0, 0);
    Vector3 W_1 = new Vector3(0, 0, 0);

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        //计算初始位置的转动惯量。$I_ref$
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            //惯性张量的正确计算？
            //Vector3 tempP = vertices[i];
            //I_ref[0, 0] += m * (tempP.y * tempP.y + tempP.z * tempP.z);
            //I_ref[1, 1] += m * (tempP.z * tempP.z + tempP.x * tempP.x);
            //I_ref[2, 2] += m * (tempP.x * tempP.x + tempP.y * tempP.y);
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    //unity 不提供矩阵减法
    Matrix4x4 MatrixMinus(Matrix4x4 m, Matrix4x4 n)
    {
        Matrix4x4 A = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                A[i, j] = m[i, j] - n[i, j];
            }
        }
        return A;
    }

    Matrix4x4 MatrixTimesFloat(Matrix4x4 m, float k)
    {
        Matrix4x4 M = m;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                M[i, j] *= k;
            }
        }
        return M;
    }

    Quaternion QuatAdd(Quaternion s, Quaternion v)
    {
        Quaternion q = new Quaternion(s.x + v.x, s.y + v.y, s.z + v.z, s.w + v.w);
        return q;
    }

    //本质就是反对称矩阵
    Matrix4x4 Convert_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Vector3 Rr_Sum_collision = new Vector3(0, 0, 0);
        Vector3 V_Sum_Collision = new Vector3(0, 0, 0);

        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        //transform是世界坐标系下,x_i是在模型坐标系下。得到模型的平移向量和旋转向量
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        Vector3 X_center = transform.position;

        uint counter = 0;
        //碰撞检测，采用比较初级的检测算法：直接遍历每个点,判断是否有点在平面内部
        for (int i = 0; i < vertices.Length; i++)
        {
            //Vector3 Vertice_i = vertices[i];
            Vector3 R_ri = R.MultiplyPoint(vertices[i]);
            Vector3 X_i = X_center + R_ri;
            //sdf求解
            if (Vector3.Dot(X_i - P, N) < 0.0f)
            {
                //𝐯𝑖 ⟵ 𝐯 + 𝛚_1 × 𝐑𝐫i  
                Vector3 V_i = V_1 + Vector3.Cross(W_1, R_ri);
                //判断速度方向是向平面内运动还是向外运动。如果是内则需要计算碰撞。
                if (Vector3.Dot(V_i, N) < 0)
                {
                    Rr_Sum_collision += R_ri;
                    V_Sum_Collision += V_i;
                    counter++;
                }
            }
        }

        //计算冲量大小更新速度，角速度
        if (counter > 0)
        {
            Rr_Sum_collision /= counter;
            V_Sum_Collision /= counter;

            //虚拟碰撞点：将速度collisonVelocity分解成水平向和垂直向，然后进行碰撞衰减计算
            Vector3 V_N = Vector3.Dot(V_Sum_Collision, N) * N;
            Vector3 V_T = V_Sum_Collision - V_N;
            //由库仑定律： a = max(1 - μt(1 + μn)||Vni||/||Vti||)
            float a = Mathf.Max(1 - friction * (1 + restitution) * V_N.magnitude / V_T.magnitude, 0);
            Vector3 V_N_new = -V_N * restitution;
            Vector3 V_T_new = a * V_T;
            Vector3 V_new = V_N_new + V_T_new;

            //compute the impulse J， 计算冲量J， 需要注意冲量和冲量矩的区别。
            Matrix4x4 I_inverse = I_ref.inverse;
            Matrix4x4 R_ri_cross = Convert_Cross_Matrix(Rr_Sum_collision);
            Matrix4x4 K = MatrixMinus(MatrixTimesFloat(Matrix4x4.identity, 1 / mass), R_ri_cross * I_inverse * R_ri_cross);
            Vector3 J = K.inverse * (V_new - V_Sum_Collision);

            //通过冲量来更新刚体的速度和角速度
            V_1 = V_1 + J / mass;
            //冲量矩 R_ri * J, 这里使用Rr_collision是因为需要对多个碰撞点去平均值计算。
            W_1 = W_1 + I_inverse.MultiplyVector(Vector3.Cross(Rr_Sum_collision, J));

            //系数μN碰撞后衰减，防止物体抖动。
            restitution *= 0.9f;
        }
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(5, 2, 0);
            w = new Vector3(0, 1, 0);
            launched = true;
            Debug.Log("hello bunny");
        }


        if (launched)
        {
            //Vector3 V_1 = v;
            //Vector3 W_1 = w;
            // Part I: Update velocities  (采用leapfrog method，先计算出速度，然后更新位置)
            //calculate torque or to update the angular velocity
            Vector3 F_gravity = new Vector3(0.0f, -mass * gravity, 0.0f);
            V_1 = v + F_gravity / mass * dt;
            V_1 *= linear_decay; //To produce damping effects ， 如果是基于物理的模拟应该是建立f(x,v)的函数来计算力。这里直接使用decay来模拟。
            W_1 = w * angular_decay;
            //只有重力不需要更新torque。gravitation being the only force, you don’t need to calculate torque or to update the angular velocity


            // Part II: Collision Impulse，输入地面和墙体的sdf， 计算碰撞。
            //碰撞之前需要更新惯性张量 I_1 = R * I_ref * R^T 
            //Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
            //Matrix4x4 I_1 = R * I_ref * Matrix4x4.Transpose(R);
            //I_ref = I_1;

            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            //更新碰撞后的速度
            v = V_1;
            w = W_1;

            // Part III: Update position & orientation, 采用leapfrog method。
            //Update linear status
            Vector3 x_0 = transform.position;
            Quaternion q_0 = transform.rotation;
            //Position_frame1 = Position_frame0 + DeltaTime * velocity
            Vector3 x_1 = x_0 + dt * V_1;
            //Update angular status ,将角速度的变化量转换成四元数相乘。
            Quaternion q_w = new Quaternion(dt * W_1.x / 2, dt * W_1.y / 2, dt * W_1.z / 2, 0.0f);
            Quaternion q_1 = QuatAdd(q_0, q_w * q_0);
            q_1 = Quaternion.Normalize(q_1);
            // Part IV: Assign to the object
            transform.position = x_1;
            transform.rotation = q_1;
        }
    }
}
