
extern "C" {
int run_luac_main(int argc, char* argv[]);
}

int main(int argc, char* argv[]) { return run_luac_main(argc, argv); }
