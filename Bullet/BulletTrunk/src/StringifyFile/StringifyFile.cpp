#include <iostream>
#include <fstream>
#include <cstring>

int main( int argc, char * argv[] )
{
	if( argc < 4 )
	{
		std::cout << "Stringify must be called as follows:\n\tstringifyFile destinationfile sourcefile nameofstring\n";
		exit(1);
	}

	char *destinationFileName = argv[1];
	char *sourceFileName = argv[2];
	char *stringName = argv[3];

	std::cerr << "Stringifying from " << sourceFileName << " to " << destinationFileName << " wrapping in " << stringName << "\n";
	
	std::fstream sourceFile( sourceFileName, std::ios_base::in );
	std::fstream destinationFile( destinationFileName, std::ios_base::out );

	destinationFile << "static const char *" << stringName << " = \\\n";

	// Fixed length here, should be safe for our purposes
	char str[2000];
	while( !sourceFile.eof() )
	{
		sourceFile.getline( str, 2000 );

		// Go through line outputting char by char and escaping things 
		// that need escaping
		int length = strlen( str );
		destinationFile << "\t\"";
		for( int i = 0; i < length; ++i )
		{
			// Skip newline characters
			if( str[i] != '\n' )
			{
				// Escape \, " and ' with \\, \", \'
				if( str[i] == '\\' || str[i] == '\"' || str[i] == '\'' )
					destinationFile << '\\';
				destinationFile << str[i];
			}
		}
		destinationFile << "\\n\"\n";
	}
	destinationFile << "\t\"\";\n";
	destinationFile.close();
	sourceFile.close();

	return 0;
}