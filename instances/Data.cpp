#include <iostream>
#include <string>
#include <fstream>  // for file operations
#include <cstdlib>  // for rand() and srand()
#include <chrono>   // for high_resolution_clock

using namespace std;

// 客户数目     10 - 20 - 40
// 采摘车数目    4 -  8 - 10
// 运输车数目    3 -  6 -  8
// 运输车载量   60 - 80 - 100

int main()
{
    for (int Num = 1; Num <= 30; Num++)
    {
        string index = to_string(Num);
        // 算例名
        string filename = "T20_" + index;
        // 文件名
        string filenamewithextension = filename + ".txt";
        // 行数 --- 等于客户数目 + 1
        const int rows = 21;
        // 采摘车数目
        const int Picking_Num = 8;
        // 运输车数目
        const int Freight_Num = 6;
        // 运输车装载量
        const int Capacity = 80;


        // 列数 --- 列数不变
        const int cols = 6;
        // 算例数据二维矩阵
        int array[rows][cols];

        // Initialize random seed with high precision time
        auto now = chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        unsigned seed = duration.count();
        srand(seed);

        // Fill the array
        for (int i = 0; i < rows; ++i)
        {
            array[i][0] = i; // First column
            for (int j = 1; j < cols; ++j)
            {
                if (j == 1 || j == 2)
                {
                    array[i][j] = rand() % 149 + 1;
                }
                // Volume
                if (j == 3)
                {
                    array[i][j] = rand() % 68 + 5;
                }
                // Buffer
                if (j == 4)
                {
                    array[i][j] = rand() % (2 * array[i][j - 1]) + array[i][j - 1] + 1;
                }
                // Service Time
                if (j == 5)
                {
                    array[i][j] = rand() % 105 + 15;
                }
            }
        }

        // Open a file for writing
        ofstream outFile(filenamewithextension);

        if (!outFile)
        {
            cout << "Error opening file for writing!" << endl;
            return 1;
        }
        outFile << filename << endl;
        outFile << endl;
        outFile << "Vehicle" << endl;
        outFile << "PickingNum\t\tDeliveryNum\t\tCapacity" << endl;
        outFile << "\t" << Picking_Num << "\t\t\t\t" << Freight_Num << "\t\t\t\t" << Capacity << endl;
        outFile << endl;
        outFile << "Customer" << endl;
        outFile << "Cus No.\t\tXCOORD.\t\tYCOORD.\t\tVolume\t\tBuffer\t\tServiceTime" << endl;
        outFile << endl;

        // Output the array
        for (int i = 0; i < rows; ++i)
        {
            outFile << "\t";
            for (int j = 0; j < cols; ++j)
            {
                outFile << array[i][j] << "\t\t";
            }
            outFile << endl;
        }

        // Close the file
        outFile.close();
    }

    return 0;
}
