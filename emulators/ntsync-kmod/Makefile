PORTNAME=	ntsync-kmod
DISTVERSION=	g20260416
CATEGORIES=	emulators

MAINTAINER=	tmp386@live.com
COMMENT=	NTSync kernel driver for Windows compatibility layers
WWW=		https://github.com/XaeroVincent/FreeBSD-NTSync

LICENSE=	GPLv2
LICENSE_FILE=	${WRKSRC}/LICENSE

USES=		compiler:c11 kmod

USE_GITHUB=	yes
GH_ACCOUNT=	XaeroVincent
GH_PROJECT=	FreeBSD-NTSync
GH_TAGNAME=	e9497ee2309526cd078dfae3b8b14b3cbf4bdac4

MAKE_ARGS=	COMPAT32=yes

PLIST_FILES=	${KMODDIR}/ntsync.ko \
		include/linux/ntsync.h

post-install:
	${MKDIR} ${STAGEDIR}${PREFIX}/include/linux
	${INSTALL_DATA} ${WRKSRC}/ntsync.h ${STAGEDIR}${PREFIX}/include/linux

.include <bsd.port.mk>
