using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

using System;
using System.IO;

public class GetUnityVersion : MonoBehaviour {

    static void GetVersionInfo ()
    {
        string[] cmdArgs = System.Environment.GetCommandLineArgs();
        string path = cmdArgs[1];

        char delim = '.';
        string[] versionInfo = Application.unityVersion.Split(delim);

        using (StreamWriter sw = new StreamWriter(path))
        {
        	foreach (string spl in versionInfo)
        	{
				sw.WriteLine(spl);
        	}
        }
    }
}
