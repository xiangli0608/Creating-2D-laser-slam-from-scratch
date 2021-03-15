#include <cstdio>
#include <windows.h>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <conio.h>
using namespace std;

char map[25][25];

void init_map()
{
    for (int j = 1; j <= 12; j++)
        map[0][j] = ' ';
    for (int i = 1; i <= 7; i++)
        for (int j = 1; j <= 12; j++)
            map[i][j] = 'O';
}

struct Player
{
    char o;
} p[3];

void init_player()
{
    p[1].o = 'A';
    p[2].o = 'B';
}

int cnt;

void out_map()
{
    system("cls");
    for (int i = 0; i <= 7; i++)
        for (int j = 1; j <= 12; j++)
        {
            cout << map[i][j];
            if (j == 12)
                cout << endl;
        }
}

int drx[9] = {0, 1, -1, 0, 0, 1, -1, 1, -1};
int dry[9] = {0, 0, 0, 1, -1, 1, -1, -1, 1};

int check()
{
    for (int i = 1; i <= 7; i++)
        for (int j = 1; j <= 12; j++)
            if (map[i][j] != 'O')
            {
                int cnt, x, y;
                for (int p = 1; p <= 8; p++)
                {
                    cnt = 0;
                    x = i;
                    y = j;
                    for (int m = 1; m <= 4; m++)
                    {
                        x += drx[p], y += dry[p];
                        if (map[x][y] == map[i][j])
                            cnt++;
                        else
                            break;
                    }
                    if (cnt == 3)
                    {
                        x = i;
                        y = j;
                        for (int m = 1; m <= 4; m++)
                        {
                            map[x][y] = '*';
                            x += drx[p], y += dry[p];
                            out_map();
                            Sleep(150);
                        }
                        if (map[i][j] == 'B')
                            return 1;
                        else
                            return 2;
                    }
                }
            }
    return 0;
}

void won(int s)
{
    char c;
    if (s == 1)
        c = 'B';
    else
        c = 'A';
    printf("Player %c has won the game !!!!\n", c);
}

int winer, round;

void godown(int y, char o)
{
    int x = 1;

    while (map[x][y] == 'O' and x <= 7)
    {
        map[x][y] = o;
        out_map();
        Sleep(150);

        map[x][y] = 'O';
        x++;
    }

    x--;
    map[x][y] = o;
    out_map();
}

int bx = 0, by = 0;

void play(int round)
{
    int k = round % 2 + 1;
    char c = getch(), o = p[k].o;

    map[bx][by] = '*';
    out_map();

    while (1)
    {
        if (c == 'A' or c == 'a' or c == 'D' or c == 'd')
        {
            map[bx][by] = ' ';
            if (c == 'A' or c == 'a')
                by--;
            else
                by++;
            if (by <= 0 or by > 12)
            {
                if (c == 'A' or c == 'a')
                    by++;
                else
                    by--;
                c = getch();
                continue;
            }
            map[bx][by] = '*';
            out_map();
        }
        else
        {
            godown(by, o);
            break;
        }
        c = getch();
    }
}

int main()
{
    init_map();
    init_player();

    out_map();

    puts("\nHave fun playing! Made by C20182030.");
    puts("Press any key to start :");
    char c = getch();

    while (1)
    {
        winer = check();
        if (winer)
        {
            won(winer);
            c = getch();
            break;
        }

        for (int j = 1; j <= 12; j++)
            map[0][j] = ' ';

        round++;
        play(round);
    }
}
