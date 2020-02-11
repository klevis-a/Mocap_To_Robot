IF EXIST parameters\parametersM20.xml DEL /F parameters\parametersM20.xml
IF EXIST parameters\robotM20.xml DEL /F parameters\robotM20.xml
IF EXIST parameters\tpProgram_humerus.xml DEL /F parameters\tpProgram_humerus.xml

mklink parameters\parametersM20.xml %1\parametersM20.xml
mklink parameters\robotM20.xml %1\robotM20.xml
mklink parameters\tpProgram_humerus.xml %1\tpProgram_humerus.xml
