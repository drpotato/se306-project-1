headerFile = "glext.h"
defineList = "glx_define_list.txt"
loaderHPPFile = "glloader.hpp"
loaderCPPFile = "glloader.cpp"

def genLoaderFromHeader():
    # Treats "#ifndef <name>" as being the start of a "section" named <name>
    sectionSig = "#ifndef "
    prototypeSig = "GLAPI "
    cascadeSig = "GL_VERSION_"
    excludedSections = set(["GL_VERSION_1_1", "GL_VERSION_1_2", "GL_VERSION_1_3", "GL_ARB_imaging"])

    lastSection = ""
    sections = {}
    cascadeSet = set()
    
    # Avoid redefining functions from GLX
    excludedPrototypes = set()
    for line in open(defineList, "r"):
        excludedPrototypes.add(line.strip())
    
    for line in open(headerFile, "r"):
        if line.startswith(sectionSig):
            lastSection = line[len(sectionSig):].split("\n")[0]

            if lastSection.startswith(cascadeSig):
                cascadeSet.add(lastSection)
            
        elif line.startswith(prototypeSig):
            prototypeName = line.split("(")[0].rstrip().split(" ")[-1]

            if prototypeName in excludedPrototypes:
                continue

            if lastSection not in sections:
                sections[lastSection] = [prototypeName]
            else:
                sections[lastSection].append(prototypeName)

    cascadeList = sorted(cascadeSet, reverse=True)

    fHppStrs = []
    fCppStrs = []
    fCppLoadStrs = []

    fHppStrs.append("#define GL_GLEXT_LEGACY\n")
    fHppStrs.append("#include <GL/glx.h>\n")
    fHppStrs.append("#include \"%s\"\n" % (headerFile))
    fHppStrs.append("#ifndef SE306P1_UPSTAGE_GLLOADER_HPP_DEFINED\n")
    fHppStrs.append("#define SE306P1_UPSTAGE_GLLOADER_HPP_DEFINED\n")
    fHppStrs.append("namespace ups { void loadOpenGL(); }\n")
    fHppStrs.append("#endif // #ifndef SE306P1_UPSTAGE_GLLOADER_HPP_DEFINED\n")
    
    fCppStrs.append("#include \"%s\"\n" % (loaderHPPFile))
    fCppStrs.append("#define loadGLX(T,N) (N=(T)glXGetProcAddress(reinterpret_cast<const GLubyte *>(#N)))\n")

    for i in range(len(cascadeList) - 1):
        fHppStrs.append("#ifdef LOAD_%s\n#define LOAD_%s\n#endif\n" % (
            cascadeList[i], cascadeList[i+1]))

    for section in sections:#sorted(sections):
        prototypes = sections[section]
        fHppStrs.append("#ifdef LOAD_%s\n" % (section))
        fHppStrs.append("#ifndef LOAD_%s_ALREADY_COMPLETED\n" % (section))
        for prototype in prototypes:#sorted(prototypes):
            prototypeType = "PFN" + prototype.upper() + "PROC"
            fHppStrs.append("extern %s %s;\n" % (prototypeType, prototype))
            fCppStrs.append("%s %s;\n" % (prototypeType, prototype))
            #fCppLoadStrs.append("loadGLX(%s,%s);\n" % (prototypeType, prototype))
            fCppLoadStrs.append("\t%s = (%s)glXGetProcAddress(reinterpret_cast<const GLubyte *>(\"%s\"));\n" % (
                prototype, prototypeType, prototype))
        fHppStrs.append("#endif// #ifndef LOAD_%s_ALREADY_COMPLETED\n" % (section))
        fHppStrs.append("#endif// #ifdef LOAD_%s\n" % (section))
            
    


    fHpp = open(loaderHPPFile, "wb")
    fHpp.write(("".join(fHppStrs)).encode())
    fHpp.close()

    fCppStrs.append("void ups::loadOpenGL()\n{\n")
    fCppStrs.extend(fCppLoadStrs)
    fCppStrs.append("}")
    fCpp = open(loaderCPPFile, "wb")
    fCpp.write(("".join(fCppStrs)).encode())
    fCpp.close()

if __name__ == "__main__":
    genLoaderFromHeader()
