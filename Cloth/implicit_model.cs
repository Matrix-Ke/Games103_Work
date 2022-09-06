using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_model : MonoBehaviour
{
    float dt = 0.0333f;
    float mass = 1;
    float damping = 0.99f;
    //float rho = 0.995f;
    float spring_k = 8000;
    int[] E; //edge
    float[] L; //spring origin length
    Vector3[] V_velocity;

    Vector3 F_gravity = new Vector3(0.0f, -9.8f, 0.0f); //��������

    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;

        //Resize the mesh.
        int n = 21;
        Vector3[] X = new Vector3[n * n];
        Vector2[] UV = new Vector2[n * n];
        int[] triangles = new int[(n - 1) * (n - 1) * 6];
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
            }
        int dt = 0;
        for (int j = 0; j < n - 1; j++)
            for (int i = 0; i < n - 1; i++)
            {
                triangles[dt * 6 + 0] = j * n + i;
                triangles[dt * 6 + 1] = j * n + i + 1;
                triangles[dt * 6 + 2] = (j + 1) * n + i + 1;
                triangles[dt * 6 + 3] = j * n + i;
                triangles[dt * 6 + 4] = (j + 1) * n + i + 1;
                triangles[dt * 6 + 5] = (j + 1) * n + i;
                dt++;
            }
        mesh.vertices = X;
        mesh.triangles = triangles;
        mesh.uv = UV;
        mesh.RecalculateNormals();


        //Construct the original E
        int[] _E = new int[triangles.Length * 2];
        for (int i = 0; i < triangles.Length; i += 3)
        {
            _E[i * 2 + 0] = triangles[i + 0];
            _E[i * 2 + 1] = triangles[i + 1];
            _E[i * 2 + 2] = triangles[i + 1];
            _E[i * 2 + 3] = triangles[i + 2];
            _E[i * 2 + 4] = triangles[i + 2];
            _E[i * 2 + 5] = triangles[i + 0];
        }
        //Reorder the original edge list
        for (int i = 0; i < _E.Length; i += 2)
            if (_E[i] > _E[i + 1])
                Swap(ref _E[i], ref _E[i + 1]);
        //Sort the original edge list using quicksort
        Quick_Sort(ref _E, 0, _E.Length / 2 - 1);

        int e_number = 0;
        for (int i = 0; i < _E.Length; i += 2)
            if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                e_number++;

        E = new int[e_number * 2];
        for (int i = 0, e = 0; i < _E.Length; i += 2)
            if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
            {
                E[e * 2 + 0] = _E[i + 0];
                E[e * 2 + 1] = _E[i + 1];
                e++;
            }

        L = new float[E.Length / 2];
        for (int e = 0; e < E.Length / 2; e++)
        {
            int v0 = E[e * 2 + 0];
            int v1 = E[e * 2 + 1];
            L[e] = (X[v0] - X[v1]).magnitude;
        }

        V_velocity = new Vector3[X.Length];
        for (int i = 0; i < V_velocity.Length; i++)
            V_velocity[i] = new Vector3(0, 0, 0);
    }

    void Quick_Sort(ref int[] a, int l, int r)
    {
        int j;
        if (l < r)
        {
            j = Quick_Sort_Partition(ref a, l, r);
            Quick_Sort(ref a, l, j - 1);
            Quick_Sort(ref a, j + 1, r);
        }
    }

    int Quick_Sort_Partition(ref int[] a, int l, int r)
    {
        int pivot_0, pivot_1, i, j;
        pivot_0 = a[l * 2 + 0];
        pivot_1 = a[l * 2 + 1];
        i = l;
        j = r + 1;
        while (true)
        {
            do ++i; while (i <= r && (a[i * 2] < pivot_0 || a[i * 2] == pivot_0 && a[i * 2 + 1] <= pivot_1));
            do --j; while (a[j * 2] > pivot_0 || a[j * 2] == pivot_0 && a[j * 2 + 1] > pivot_1);
            if (i >= j) break;
            Swap(ref a[i * 2], ref a[j * 2]);
            Swap(ref a[i * 2 + 1], ref a[j * 2 + 1]);
        }
        Swap(ref a[l * 2 + 0], ref a[j * 2 + 0]);
        Swap(ref a[l * 2 + 1], ref a[j * 2 + 1]);
        return j;
    }

    void Swap(ref int a, ref int b)
    {
        int temp = a;
        a = b;
        b = temp;
    }

    void Collision_Handling()
    {
        float r = 2.7f;
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;

        //Handle colllision, detect collision and apply impulse if needed.
        GameObject sphere = GameObject.Find("Sphere");
        Vector3 c = sphere.transform.position;

        for (int i = 0; i < X.Length; i++)
        {
            Vector3 xi_c = X[i] - c;
            if (xi_c.magnitude < r)
            {
                V_velocity[i] += (c + r * xi_c / xi_c.magnitude - X[i]) / dt;
                X[i] = c + r * xi_c / xi_c.magnitude;
            }
        }
        mesh.vertices = X;
    }

    void Get_Gradient(Vector3[] X, Vector3[] X_hat, float dt, Vector3[] G)
    {
        //Momentum and Gravity. ��������
        for (int i = 0; i < X.Length; i++)
        {
            G[i] = 1 / (dt * dt) * mass * (X[i] - X_hat[i]) - mass * F_gravity;
        }


        //Spring Force. ���ӵ�����
        uint E_number = (uint)(E.Length / 2);
        for (int e = 0; e < E_number; e++)
        {
            int i_index = E[e * 2 + 0];
            int j_index = E[e * 2 + 1];
            Vector3 X_i = X[i_index];
            Vector3 X_j = X[j_index];
            Vector3 f = -spring_k * (1 - L[e] / (X_i - X_j).magnitude) * (X_i - X_j);
            G[i_index] -= f;
            G[j_index] += f;
        }
    }


    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;
        Vector3[] last_X = new Vector3[X.Length];
        Vector3[] X_hat = new Vector3[X.Length];
        Vector3[] G_gradient = new Vector3[X.Length];

        //Initial Setup.
        for (int i = 0; i < X.Length; i++)
        {
            V_velocity[i] *= damping;
            X_hat[i] = X[i] + V_velocity[i] * dt;
            X[i] = X_hat[i]; //x_i�ǳ����²⣬���������ĸ���.(��ʼ״̬������Ϊ0��ֱ�����ٶ�ȥ����һ�ξ����ټ�����µ��ٶȣ�Ȼ�������ʽŷ������)
            //G[i] = mass * F_gravity;
        }

        //��������32�ε�����һ����˵����Խ��Ч��Խ��
        for (int k = 0; k < 32; k++)
        {
            Get_Gradient(X, X_hat, dt, G_gradient);

            //Update X by gradient.�� Hessian ��Ϊ�ԽǾ�����ѡ����򵥵ķ���
            for (int i = 0; i < X.Length; i++)
            {
                if (i == 0 || i == 20) continue; //����̶�

                //x_i = x_i-(\frac{1}{\Delta t^2} m_i+4 k)^{-1} g_i\\
                X[i] -= G_gradient[i] / (mass / (dt * dt) + 4 * spring_k);
            }
        }



        //Finishing.
        for (int i = 0; i < X.Length; i++)
        {
            //����̶�
            if (i == 0 || i == 20) continue;
            V_velocity[i] += (X[i] - X_hat[i]) / dt;
        }

        mesh.vertices = X;

        Collision_Handling();
        mesh.RecalculateNormals();
    }
}
