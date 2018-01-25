#ifndef UTILS_H
#define UTILS_H
struct point
{
    int x;
    int y;
};


class Utils{
public:
    static int initialize_randomness(int seed)
    { // initializes the seed, if seed is -1 then it uses system time as seed
        int seed1;
        time_t* tp;
        tp = NULL;
        if(seed == -1)
            seed1 = time(tp);
        else
            seed1 = seed;
        srand(seed1);
        return seed1;
    }

    static int randint(int lower, int upper)
    { // returns a random integer between lower and upper
        return lower + rand() % (upper - lower + 1);
    }
};

#endif // UTILS_H
