using PicoGK;

string outputPath = args.Length > 0
    ? Path.GetFullPath(args[0])
    : Path.Combine(Directory.GetCurrentDirectory(), "PicoGK.stl");

string? outputDirectory = Path.GetDirectoryName(outputPath);
if (!string.IsNullOrEmpty(outputDirectory))
{
    Directory.CreateDirectory(outputDirectory);
}

try
{
    using Library library = new(0.1f);
    Voxels voxels = new(Utils.mshCreateCube());
    voxels.mshAsMesh().SaveToStlFile(outputPath);

    Console.WriteLine($"STL generated: {outputPath}");
}
catch (Exception ex)
{
    Console.Error.WriteLine($"Failed to generate STL file at: {outputPath}");
    Console.Error.WriteLine(ex.Message);
    Environment.ExitCode = 1;
}
