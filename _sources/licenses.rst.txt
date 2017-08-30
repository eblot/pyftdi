Licenses
--------

.. include:: defs.rst

For historical reasons (PyFtdi has been initially developed as a compatibility
layer with libftdi_), the main ``ftdi.py`` file has been license under the same
license as the libftdi_ project, the GNU Lesser General Public License LGPL v2
license. It does not share code from this project anymore, but implements some
parts of the same API.

It's really hard to figure out what implies this license with the Python
language. If you can, let me know :-) It is also quite complex to change the
license now, so PyFtdi has to live with it.

Anyway wherease the ``ftdi.py`` file has been kept under the LGPL v2 license,
all other files are released under a less controversial and less
nearly-impossible-to-understand licensing scheme: the MIT license.

From my perspective, you may use it freely in open source or close source, free
or commercial projects as long as you comply with the MIT license. However,
I'm not a lawyer so I can't help you with the implications of the LGPL v2
license in a Python-based close-source project, check out for yourself to be
sure you comply with the ``ftdi.py`` file license, which is always required
whatever the actual use of PyFtdi_.

LGPL v2
~~~~~~~

.. code-block:: none

  Copyright (c) 2008-2017 Emmanuel Blot <emmanuel.blot@free.fr>
  All Rights Reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


MIT
~~~

.. code-block:: none

  Copyright (c) 2008-2017 Emmanuel Blot <emmanuel.blot@free.fr>
  All Rights Reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Neotion nor the names of its contributors may
        be used to endorse or promote products derived from this software
        without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL NEOTION BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
