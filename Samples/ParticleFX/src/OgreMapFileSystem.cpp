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

#include "OgreMapFileSystem.h"

#include "OgreConfigFile.h"

namespace Ogre {

    //-----------------------------------------------------------------------
    MapFileSystemArchive::MapFileSystemArchive(const String& name, const String& fsPath, const String& archType, bool readOnly )
        : FileSystemArchive(fsPath, archType,readOnly), mMapFileName(name)
    {
        
    }
    
    //-----------------------------------------------------------------------
    void MapFileSystemArchive::findFiles(const String& pattern, bool recursive, 
        bool dirs, StringVector* simpleList, FileInfoList* detailList)
    {
        FileMapConstIt beginIt = mFileMap.begin();
        FileMapConstIt endIt = mFileMap.end();
        for (FileMapConstIt it = beginIt; it!=endIt; it++)
        {
            const String& srcName = it->first;
            const String& dstName = it->second;
            
            if (!StringUtil::match(srcName, pattern, false))
                continue;
            
            if (simpleList)
            {
                //if (FileSystemArchive::exists(dstName))
                simpleList->push_back(srcName);
            }
            else if (detailList)
            {
                FileInfoListPtr fiList = FileSystemArchive::findFileInfo(dstName);
                if (fiList->size()<=0) continue;
                
                FileInfo fi = *(fiList->begin());
                fi.filename = srcName;
                StringUtil::splitFilename(fi.filename, fi.basename, fi.path);
                detailList->push_back(fi);
            }
            
        }
    }
    //-----------------------------------------------------------------------
    MapFileSystemArchive::~MapFileSystemArchive()
    {
        unload();
    }
    
    bool MapFileSystemArchive::findFileFromMap(const String& srcFile,String& dstFile) const
    {
        FileMapConstIt findIt = mFileMap.find(srcFile);
        if(findIt != mFileMap.end())
        {
            dstFile = findIt->second;
            return true;
        }
        return false;
    }
    
    //-----------------------------------------------------------------------
    void MapFileSystemArchive::load()
    {
		// nothing to do here
        
        Ogre::String secName, srcName, dstName;
        Ogre::ConfigFile cf;
        cf.load(mMapFileName);
        
        Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
        while (seci.hasMoreElements())
        {
            secName = seci.peekNextKey();
            Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
            Ogre::ConfigFile::SettingsMultiMap::iterator i;
            for (i = settings->begin(); i != settings->end(); ++i)
            {
                srcName = i->first;
                dstName = i->second;
                mFileMap[srcName] = dstName;
            }
        }

    }
    //-----------------------------------------------------------------------
    void MapFileSystemArchive::unload()
    {
        // nothing to see here, move along
        mFileMap.clear();
    }
    //-----------------------------------------------------------------------
    DataStreamPtr MapFileSystemArchive::open(const String& filename, bool readOnly)
    {
        String dstName;
        bool findFile = findFileFromMap(filename, dstName);
        if (findFile) {
            return FileSystemArchive::open(dstName, readOnly);
        }
        return DataStreamPtr();
    }
	//---------------------------------------------------------------------
	DataStreamPtr MapFileSystemArchive::create(const String& filename)
    {
        String dstName;
        bool findFile = findFileFromMap(filename, dstName);
        if (findFile) {
            return FileSystemArchive::create(dstName);
        }
        return DataStreamPtr();
	}
	//---------------------------------------------------------------------
	void MapFileSystemArchive::remove(const String& filename)
    {
        String dstName;
        bool findFile = findFileFromMap(filename, dstName);
        if (findFile) {
            FileSystemArchive::remove(dstName);
        }

	}
    //-----------------------------------------------------------------------
    StringVectorPtr MapFileSystemArchive::list(bool recursive, bool dirs)
    {
		// directory change requires locking due to saved returns
		// Note that we have to tell the SharedPtr to use OGRE_DELETE_T not OGRE_DELETE by passing category
		StringVectorPtr ret(OGRE_NEW_T(StringVector, MEMCATEGORY_GENERAL)(), SPFM_DELETE_T);

        findFiles("*", recursive, dirs, ret.getPointer(), 0);

        return ret;
    }
    //-----------------------------------------------------------------------
    FileInfoListPtr MapFileSystemArchive::listFileInfo(bool recursive, bool dirs)
    {
		// Note that we have to tell the SharedPtr to use OGRE_DELETE_T not OGRE_DELETE by passing category
        FileInfoListPtr ret(OGRE_NEW_T(FileInfoList, MEMCATEGORY_GENERAL)(), SPFM_DELETE_T);

        findFiles("*", recursive, dirs, 0, ret.getPointer());

        return ret;
    }
    //-----------------------------------------------------------------------
    StringVectorPtr MapFileSystemArchive::find(const String& pattern,
                                            bool recursive, bool dirs)
    {
		// Note that we have to tell the SharedPtr to use OGRE_DELETE_T not OGRE_DELETE by passing category
		StringVectorPtr ret(OGRE_NEW_T(StringVector, MEMCATEGORY_GENERAL)(), SPFM_DELETE_T);

        findFiles(pattern, recursive, dirs, ret.getPointer(), 0);

        return ret;

    }
    //-----------------------------------------------------------------------
    FileInfoListPtr MapFileSystemArchive::findFileInfo(const String& pattern, 
        bool recursive, bool dirs)
    {
		// Note that we have to tell the SharedPtr to use OGRE_DELETE_T not OGRE_DELETE by passing category
		FileInfoListPtr ret(OGRE_NEW_T(FileInfoList, MEMCATEGORY_GENERAL)(), SPFM_DELETE_T);

        findFiles(pattern, recursive, dirs, 0, ret.getPointer());

        return ret;
    }
    //-----------------------------------------------------------------------
	bool MapFileSystemArchive::exists(const String& filename)
	{
        
        String dstName;
        bool findFile = findFileFromMap(filename, dstName);
        if (findFile) {
            return FileSystemArchive::exists(dstName);
        }
        return false;
	}
	//---------------------------------------------------------------------
	time_t MapFileSystemArchive::getModifiedTime(const String& filename)
	{        
        String dstName;
        bool findFile = findFileFromMap(filename, dstName);
        if (findFile) {
            return FileSystemArchive::getModifiedTime(dstName);
        }
        return 0;

	}
    //-----------------------------------------------------------------------
    const String& MapFileSystemArchiveFactory::getType(void) const
    {
        static String name = "MapFileSystem";
        return name;
    }

}
// jianshun MapFileSystem 20150313 end
