/* -*- c++ -*- */
/*
 * Gqrx SDR: Software defined radio receiver powered by GNU Radio and Qt
 *           http://gqrx.dk/
 *
 * Copyright 2013 Christian Lindner DL2VCL, Stefano Leucci.
 *
 * Gqrx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Gqrx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef BOOKMARKS_H
#define BOOKMARKS_H

#include <QtGlobal>
#include <QString>
#include <QMap>
#include <QList>
#include <QStringList>
#include <QColor>
#include <QDebug>
#include <QSemaphore>
struct TagInfo
{
    QString name;
    QColor color;
    bool active;

    static const QColor DefaultColor;
    static const QString strUntagged;

    TagInfo()
    {
        active=true;
        this->color=DefaultColor;
    }
    TagInfo(QString name)
    {
        active=true;
        this->color=DefaultColor;
        this->name = name;
    }
};

struct BookmarkInfo
{
    qint64  frequency;
    QString name;
    QString modulation;
    qint64  bandwidth;
    QList<TagInfo*> tags;
    QColor color;
    bool userFreq ;
    int sched_key ; // see whatson.cpp

    BookmarkInfo()
    {
        this->frequency = 0;
        this->bandwidth = 0;
        this->color=DefaultColor;
        this->sched_key = 0 ;
        this->userFreq = false ;
    }

    bool operator<(const BookmarkInfo &other) const
    {
        return frequency < other.frequency;
    }
    static const QColor DefaultColor;
    const QColor GetColor() const;
    bool IsActive() const;
};

class Bookmarks : public QObject
{
    Q_OBJECT
public:
    static Bookmarks& getInstance()  {
        static Bookmarks instance;
        return instance;
    }

    void add(BookmarkInfo& info);
    void remove(int index);
    bool load();
    bool save();
    int size() { return m_BookmarkList.size(); }
    BookmarkInfo& getBookmark(int i) { return m_BookmarkList[i]; }
    QList<BookmarkInfo> getBookmarksInRange(qint64 low, qint64 high, float maxDistance=0);
    //int lowerBound(qint64 low);
    //int upperBound(qint64 high);

    QList<TagInfo> getTagList() { return  QList<TagInfo>(m_TagList); }
    TagInfo& findOrAddTag(QString tagName);
    int getTagIndex(QString tagName);
    bool removeTag(QString tagName);
    bool setTagChecked(QString tagName, bool bChecked);

    void setConfigDir(const QString&);

    void clearWhatson(qint64 start, qint64 stop);
    void clearUser();

private:

    QList<BookmarkInfo> m_BookmarkList;
    QList<TagInfo> m_TagList;
    QString        m_bookmarksFile;
    QSemaphore *sem ;

    Bookmarks(); // Singleton Constructor is private.
    Bookmarks(const Bookmarks &); // hide copy constructor
    Bookmarks& operator=(const Bookmarks &);


signals:
    void BookmarksChanged(void);
    void TagListChanged(void);
};

#endif // BOOKMARKS_H
