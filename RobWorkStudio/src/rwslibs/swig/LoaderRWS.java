package dk.robwork;

/**
 * Automatically class generated by CMake for convenient loading of the native RobWorkStudio library.
 * Before invoking any functions in the Java API, the static load function in this class should be called.
 */
public class LoaderRWS {
	
	/**
	 * Load the native library required for the Java API by using the classpath.
	 */
	public static void load(){
		System.loadLibrary(getShortName());
	}
	
	/**
	 * Load the native library required for the Java API located at specific path.
	 * @param path the path where the library can be found.
	 */
	public static void load(String path){
		System.load(path + "/" + getFullName());
	}
	
	/**
	 * Get the platform-independent name of the native library without path.
	 * @return the name of the native library.
	 */
	public static String getShortName(){
		return "rws_jni";
	}
	
	/**
	 * Get the platform-dependent name of the native library without path.
	 * @return the name of the native library.
	 */
	public static String getFullName() {
		return System.mapLibraryName(getShortName());
	}
}