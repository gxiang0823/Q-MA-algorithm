#include <iostream>
#include <string>
#include <fstream>  // for file operations
#include <cstdlib>  // for rand() and srand()
#include <chrono>   // for high_resolution_clock

using namespace std;

// �ͻ���Ŀ     10 - 20 - 40
// ��ժ����Ŀ    4 -  8 - 10
// ���䳵��Ŀ    3 -  6 -  8
// ���䳵����   60 - 80 - 100

int main()
{
    for (int Num = 1; Num <= 30; Num++)
    {
        string index = to_string(Num);
        // ������
        string filename = "T20_" + index;
        // �ļ���
        string filenamewithextension = filename + ".txt";
        // ���� --- ���ڿͻ���Ŀ + 1
        const int rows = 21;
        // ��ժ����Ŀ
        const int Picking_Num = 8;
        // ���䳵��Ŀ
        const int Freight_Num = 6;
        // ���䳵װ����
        const int Capacity = 80;


        // ���� --- ��������
        const int cols = 6;
        // �������ݶ�ά����
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
