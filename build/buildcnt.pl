#!c:/perl/bin/perl.exe

my $buildcount = 0;

open my $ifp,"<./src/main/buildcnt.cpp";
while (<$ifp>) {
    if ($_ =~ m/buildCount\s*=\s*(\d+);/) {
        $buildcount = int($1);
    }
}
close $ifp;

$buildcount++;

open my $ofp,">./src/main/buildcnt.cpp";
printf $ofp ("// buildcnt.cpp\n");
printf $ofp ("// Auto Generated\n");
printf $ofp ("volatile int buildCount = %d;\n", $buildcount);
printf $ofp ("volatile char absoluteBuildTime[] = __TIMESTAMP__;\n", $buildcount);

close $ofp;

print "New build count: $buildcount\n";
