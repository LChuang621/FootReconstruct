if(Test-Path '.\build')
{
	Remove-Item -Path '.\build' -Recurse -Force
}

New-Item -ItemType Directory -Path '.\build'

cd build

cmake ..

cmd /C "$env:VSPATH\MSBuild\Current\Bin\MSBuild.exe" .\FootReconstruct.sln /p:Configuration=Release

cd ..