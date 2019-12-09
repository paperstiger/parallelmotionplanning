/*
Define the entry point for kino rrt class
*/
int main_pendulum(int argc, char *argv[]);
int main_ao_pendulum(int argc, char* argv[]);
int main_acrobot(int argc, char* argv[]);
int main_ao_acrobot(int argc, char* argv[]);

int main(int argc, char* argv[]) {
    //main_pendulum(argc, argv);
    //main_acrobot(argc, argv);
    //main_ao_pendulum(argc, argv);
    main_ao_acrobot(argc, argv);
    return 0;
}