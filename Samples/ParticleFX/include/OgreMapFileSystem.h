/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2014 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
 */
// jianshun MapFileSystem 20150313 begin
#ifndef __MapFileSystem_H__
#define __MapFileSystem_H__

#include "OgreFileSystem.h"

namespace Ogre {
    
    
    /** \addtogroup Core
    *  @{
    */
    /** \addtogroup Resources
    *  @{
    */
    /** Specialisation of the Archive class to allow reading of files from 
        filesystem folders / directories.
    */
    class _OgreExport MapFileSystemArchive : public FileSystemArchive
    {
    protected:
        String mMapFileName;
        
        typedef std::map<String,String> FileMap;
        typedef FileMap::iterator FileMapIt;
        typedef FileMap::const_iterator FileMapConstIt;
        FileMap mFileMap;
        
        bool findFileFromMap(const String& srcFile,String& dstFile) const;
        void findFiles(const String& pattern, bool recursive, bool dirs,
            StringVector* simpleList, FileInfoList* detailList);
        

        OGRE_AUTO_MUTEX;
    public:
        MapFileSystemArchive(const String& name,const String& fsPath, const String& archType, bool readOnly );
        ~MapFileSystemArchive();

        /// @copydoc Archive::load
        void load();
        /// @copydoc Archive::unload
        void unload();

        /// @copydoc Archive::open
        DataStreamPtr open(const String& filename, bool readOnly = true);

        /// @copydoc Archive::create
        DataStreamPtr create(const String& filename);

        /// @copydoc Archive::remove
        void remove(const String& filename);

        /// @copydoc Archive::list
        StringVectorPtr list(bool recursive = true, bool dirs = false);

        /// @copydoc Archive::listFileInfo
        FileInfoListPtr listFileInfo(bool recursive = true, bool dirs = false);

        /// @copydoc Archive::find
        StringVectorPtr find(const String& pattern, bool recursive = true,
            bool dirs = false);

        /// @copydoc Archive::findFileInfo
        FileInfoListPtr findFileInfo(const String& pattern, bool recursive = true,
            bool dirs = false);

        /// @copydoc Archive::exists
        bool exists(const String& filename);

        /// @copydoc Archive::getModifiedTime
        time_t getModifiedTime(const String& filename);

    };

    /** Specialisation of ArchiveFactory for FileSystem files. */
    class _OgreExport MapFileSystemArchiveFactory : public ArchiveFactory
    {
    protected:
        String mMapBaseDir;
    public:
        MapFileSystemArchiveFactory(const String& baseDir="")
        {
            setBaseDir(baseDir);
        }
        
        virtual ~MapFileSystemArchiveFactory() {}
        /// @copydoc FactoryObj::getType
        const String& getType(void) const;
        /// @copydoc FactoryObj::createInstance
        Archive *createInstance( const String& name, bool readOnly ) 
        {
            return OGRE_NEW MapFileSystemArchive(name, mMapBaseDir, "MapFileSystem", readOnly);
        }
        /// @copydoc FactoryObj::destroyInstance
        void destroyInstance(Archive* ptr) { OGRE_DELETE ptr; }
        
        const String& getBaseDir(){return mMapBaseDir;}
        
        void setBaseDir(const String& baseDir){mMapBaseDir=baseDir;}
    };

    /** @} */
    /** @} */

} // namespace Ogre

#endif // __FileSystem_H__
// jianshun MapFileSystem 20150313 end
