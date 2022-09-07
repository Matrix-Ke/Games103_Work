using UnityEngine;
using System.Collections;

public class PBD_model : MonoBehaviour
{

    float dt = 0.0333f;
    float damping = 0.99f;
    float alpha = 0.2f; //jacobi迭代中的应变限制参数
    int[] E;
    float[] L;
    Vector3[] V_velocity;
    Vector3 F_gravity = new Vector3(0.0f, -9.8f, 0.0f); //重力向量
    Vector3[] Sum_X;
    int[] Sum_N;

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;

        //Resize the mesh.
        int n = 21;
        Vector3[] X = new Vector3[n * n];
        Vector2[] UV = new Vector2[n * n];
        int[] T = new int[(n - 1) * (n - 1) * 6];
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
            }
        int t = 0;
        for (int j = 0; j < n - 1; j++)
            for (int i = 0; i < n - 1; i++)
            {
                T[t * 6 + 0] = j * n + i;
                T[t * 6 + 1] = j * n + i + 1;
                T[t * 6 + 2] = (j + 1) * n + i + 1;
                T[t * 6 + 3] = j * n + i;
                T[t * 6 + 4] = (j + 1) * n + i + 1;
                T[t * 6 + 5] = (j + 1) * n + i;
                t++;
            }
        mesh.vertices = X;
        mesh.triangles = T;
        mesh.uv = UV;
        mesh.RecalculateNormals();

        //Construct the original edge list
        int[] _E = new int[T.Length * 2];
        for (int i = 0; i < T.Length; i += 3)
        {
            _E[i * 2 + 0] = T[i + 0];
            _E[i * 2 + 1] = T[i + 1];
            _E[i * 2 + 2] = T[i + 1];
            _E[i * 2 + 3] = T[i + 2];
            _E[i * 2 + 4] = T[i + 2];
            _E[i * 2 + 5] = T[i + 0];
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
            int i = E[e * 2 + 0];
            int j = E[e * 2 + 1];
            L[e] = (X[i] - X[j]).magnitude;
        }

        V_velocity = new Vector3[X.Length];
        for (int i = 0; i < X.Length; i++)
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

    void Strain_Limiting()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        //Apply PBD here.
        uint E_number = (uint)(E.Length / 2);
        for (int e = 0; e < E_number; e++)
        {
            //为了公式直观，代码不做任务优化
            int X_index_i = E[e * 2];
            int X_index_j = E[e * 2 + 1];
            Vector3 distanceX_ij = vertices[X_index_i] - vertices[X_index_j];
            //distanceX_ij.Normalize();
            Sum_X[X_index_i] += 0.5f * (vertices[X_index_i] + vertices[X_index_j] + L[e] * (distanceX_ij / distanceX_ij.magnitude));
            Sum_X[X_index_j] += 0.5f * (vertices[X_index_i] + vertices[X_index_j] - L[e] * (distanceX_ij / distanceX_ij.magnitude));
            Sum_N[X_index_i] += 1;
            Sum_N[X_index_j] += 1;
        }

        for (int i = 0; i < vertices.Length; i++)
        {
            //jacobi迭代,减少边顺序带来的bias。 
            if (i == 0 || i == 20) continue;
            V_velocity[i] += ((alpha * vertices[i] + Sum_X[i]) / (alpha + Sum_N[i]) - vertices[i]) / dt;
            vertices[i] = (alpha * vertices[i] + Sum_X[i]) / (alpha + Sum_N[i]);

            //rest  Sum_X 和 Sum_N
            Sum_X[i] = new Vector3(0.0f, 0.0f, 0.0f);
            Sum_N[i] = 0;
        }

        mesh.vertices = vertices;
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

    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;

        //存储顶点位置更新和顶点计数更新的总和
        Sum_X = new Vector3[X.Length];
        Sum_N = new int[X.Length];

        for (int i = 0; i < X.Length; i++)
        {
            if (i == 0 || i == 20) continue;
            //Initial Setup

            V_velocity[i] *= damping;
            V_velocity[i] += dt * F_gravity;

            X[i] += dt * V_velocity[i];
            //reset  Sum_X Sum_N
            Sum_X[i] = new Vector3(0, 0, 0);
            Sum_N[i] = 0;
        }

        mesh.vertices = X;

        for (int l = 0; l < 32; l++)
            Strain_Limiting();

        Collision_Handling();

        mesh.RecalculateNormals();
    }
}

