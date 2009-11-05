
extern "C" {
int run_lua_main(int argc, char **argv);
}

int main(int argc, char **argv) { return run_lua_main(argc, argv); }
